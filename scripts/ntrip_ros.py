#!/usr/bin/env python3

import os
import sys
import json

import rclpy
from std_msgs.msg import String

from ntrip_ros_base import NTRIPRosBase, _RTCM_MSGS_NAME
from ntrip_client.ntrip_client import NTRIPClient
from ntrip_client.nmea_parser import NMEA_DEFAULT_MAX_LENGTH, NMEA_DEFAULT_MIN_LENGTH

class NTRIPRos(NTRIPRosBase):
  def __init__(self):
    # Init the node and declare params
    super().__init__('ntrip_client')
    self.declare_parameters(
      namespace='',
      parameters=[
        ('host', '127.0.0.1'),
        ('port', 2101),
        ('mountpoint', 'mount'),
        ('ntrip_version', 'None'),
        ('user_agent', NTRIPClient.DEFAULT_USER_AGENT),
        ('authenticate', False),
        ('username', ''),
        ('password', ''),
        ('ssl', False),
        ('cert', 'None'),
        ('key', 'None'),
        ('ca_cert', 'None'),
        ('ntrip_server_hz', 1), # set send_nmea() to 1hz
        ('send_nmea', True),
        ('reconnect_attempt_wait_max_seconds', NTRIPClient.DEFAULT_RECONNECT_ATTEMPT_WAIT_MAX_SECONDS),
        ('rtcm_timeout_seconds', NTRIPClient.DEFAULT_RTCM_TIMEOUT_SECONDS),
      ]
    )

    # Read some mandatory config
    host = self.get_parameter('host').value
    port = self.get_parameter('port').value
    mountpoint = self.get_parameter('mountpoint').value

    # Optionally get the ntrip version from the launch file
    ntrip_version = self.get_parameter('ntrip_version').value
    if ntrip_version == 'None':
      ntrip_version = None

    # User-Agent presented to the caster. rtk2go blocks the stock signature, so this
    # is configurable; an empty/'None' value falls back to the client default.
    user_agent = self.get_parameter('user_agent').value
    if not user_agent or user_agent == 'None':
      user_agent = NTRIPClient.DEFAULT_USER_AGENT

    # Set the rate at which RTCM requests and NMEA messages are sent
    self.rtcm_request_rate = 1.0 / self.get_parameter('ntrip_server_hz').value

    # Whether to forward NMEA from the 'nmea' topic up to the caster. Only needed for
    # virtual/relayed (VRS) mountpoints; disable for plain base stations to avoid the
    # idle subscriber cost and uploading position to the caster. If the caster is rtk2go,
    # exit with an error if send_nmea is true, since rtk2go will ban the IP and possibly
    # this client if it receives persistent NMEA during error conditions.
    self._send_nmea = self.get_parameter('send_nmea').value
    if self._send_nmea and ((host == "rtk2go.com") or (host == "3.143.243.81")):
      self.get_logger().error('rtk2go blocks clients that send NMEA excessively, but send_nmea is true and host is rtk2go; exiting to avoid IP ban. Set send_nmea to false to fix this.')
      sys.exit(1)

    # Initialize variables to store the most recent NMEA message
    self._latest_nmea = None

    # Set the log level to debug if debug is true
    if self._debug:
      rclpy.logging.set_logger_level(self.get_logger().name, rclpy.logging.LoggingSeverity.DEBUG)

    # If we were asked to authenticate, read the username and password
    username = None
    password = None
    if self.get_parameter('authenticate').value:
      username = self.get_parameter('username').value
      password = self.get_parameter('password').value
      if not username:
        self.get_logger().error('Requested to authenticate, but param "username" was not set')
        sys.exit(1)
      if not password:
        self.get_logger().error('Requested to authenticate, but param "password" was not set')
        sys.exit(1)

    # Setup a server frequency confirmation publisher
    self._rate_confirm_pub = self.create_publisher(String, 'ntrip_server_hz', 10)

    # Initialize the client
    self._client = NTRIPClient(
      host=host,
      port=port,
      mountpoint=mountpoint,
      ntrip_version=ntrip_version,
      username=username,
      password=password,
      user_agent=user_agent,
      logerr=self.get_logger().error,
      logwarn=self.get_logger().warning,
      loginfo=self.get_logger().info,
      logdebug=self.get_logger().debug
    )

    # Get some SSL parameters for the NTRIP client
    self._client.ssl = self.get_parameter('ssl').value
    self._client.cert = self.get_parameter('cert').value
    self._client.key = self.get_parameter('key').value
    self._client.ca_cert = self.get_parameter('ca_cert').value
    if self._client.cert == 'None':
      self._client.cert = None
    if self._client.key == 'None':
      self._client.key = None
    if self._client.ca_cert == 'None':
      self._client.ca_cert = None

    # Get some timeout parameters for the NTRIP client
    self._client.nmea_parser.nmea_max_length = self._nmea_max_length
    self._client.nmea_parser.nmea_min_length = self._nmea_min_length
    self._client.reconnect_attempt_max = self._reconnect_attempt_max
    self._client.reconnect_attempt_wait_seconds = self._reconnect_attempt_wait_seconds
    self._client.reconnect_attempt_wait_max_seconds = self.get_parameter('reconnect_attempt_wait_max_seconds').value
    self._client.rtcm_timeout_seconds = self.get_parameter('rtcm_timeout_seconds').value

  # override run() in the base class with a version that retries reconnect and that only
  # subscribes to nmea and fix if needed
  def run(self):
    # Attempt initial connection; if it fails, enter backoff retry instead of exiting
    if not self._client.connect():
      self.get_logger().warning('Initial connection to NTRIP server failed, will retry with backoff')
      self._client.request_reconnect(reason='Initial connection failed')

    # Setup the subscriber for NMEA and fix data, unless NMEA forwarding is disabled
    self._nmea_sub = None
    self._fix_sub = None
    if self._send_nmea:
      self._nmea_sub = self.create_subscription(Sentence, 'nmea', self.subscribe_nmea, 10)
      self._fix_sub = self.create_subscription(NavSatFix, 'fix', self.subscribe_fix, 10)
    else:
      self.get_logger().info('send_nmea is false; not subscribing to NMEA or fix or forwarding nmea to the caster')

    # Start the timer that will send both RTCM and NMEA data at the configured rate
    self._rtcm_timer = self.create_timer(self.rtcm_request_rate, self.send_rtcm_and_nmea)

    return True

  # override subscribe_nmea() with version that works with reconnects
  def subscribe_nmea(self, nmea):
    # Cache the latest NMEA sentence
    self._latest_nmea = nmea.sentence

  def send_rtcm_and_nmea(self):
    # Request and publish RTCM data (also drives reconnect attempts)
    for raw_rtcm in self._client.recv_rtcm():
      self._rtcm_pub.publish(self._create_rtcm_message(raw_rtcm))

    # Send cached NMEA data if enabled and connected (skip during reconnect to avoid log spam)
    if self._send_nmea and self._latest_nmea is not None and not self._client.reconnecting:
      self._client.send_nmea(self._latest_nmea)

    # Publish a confirmation message to indicate the send_rtcm_and_nmea call
    confirmation_msg = String()
    confirmation_msg.data = "RTCM and NMEA sent at rate: {} Hz".format(1.0 / self.rtcm_request_rate)
    self._rate_confirm_pub.publish(confirmation_msg)

if __name__ == '__main__':
  # Start the node
  rclpy.init()
  node = NTRIPRos()
  if not node.run():
    sys.exit(1)
  try:
    # Spin until we are shut down
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  except BaseException as e:
    raise e
  finally:
    node.stop()
    
    # Shutdown the node and stop rclpy
    rclpy.shutdown()
