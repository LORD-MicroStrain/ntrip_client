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
        ('ntrip_server_hz', 1), # set to 1hz for rtk2go, override if needed
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
    self._client.rtcm_timeout_seconds = self.get_parameter('rtcm_timeout_seconds').value

if __name__ == '__main__':
  # Start the node
  rclpy.init()
  node = NTRIPRos()
  if not node.run(node.rtcm_request_rate):
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
