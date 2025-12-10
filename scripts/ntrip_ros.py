#!/usr/bin/env python

import os
import sys
import json

import rclpy

from ntrip_ros_base import NTRIPRosBase
from ntrip_client.ntrip_client import NTRIPClient

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
        ('authenticate', False),
        ('username', ''),
        ('password', ''),
        ('ssl', False),
        ('cert', 'None'),
        ('key', 'None'),
        ('ca_cert', 'None'),
        ('rtcm_timeout_seconds', NTRIPClient.DEFAULT_RTCM_TIMEOUT_SECONDS),
      ]
    )

    # Initialize all internal variables in constructor
    # Will be loaded in 'load_parameters' function.
    self.host = None
    self.port = None
    self.mountpoint = None
    self.ntrip_version = None
    self.authenticate = None
    self.username = None
    self.password = None
    self.ssl = None
    self.cert = None
    self.key = None
    self.ca_cert = None
    self.rtcm_timeout_seconds = None

    self.load_parameters()

    # Initialize the client
    self._client = self.init_ntrip_client()
    self.run()


  def load_parameters(self):
    """Load ROS parameters."""
    # Read some mandatory config
    self.host = self.get_parameter('host').value
    self.port = self.get_parameter('port').value
    self.mountpoint = self.get_parameter('mountpoint').value

    # Optionally get the ntrip version from the launch file
    self.ntrip_version = self.get_parameter('ntrip_version').value
    if self.ntrip_version == 'None':
      self.ntrip_version = None

    # If we were asked to authenticate, read the username and password
    self.username = None
    self.password = None
    self.authenticate = self.get_parameter('authenticate').value
    if self.authenticate:
      self.username = self.get_parameter('username').value
      self.password = self.get_parameter('password').value
      if not self.username or not self.password:
        raise ValueError(f'Invalid username/password: {self.username}/{self.password}')

    self.ssl = self.get_parameter('ssl').value
    self.cert = self.get_parameter('cert').value
    if self.cert == 'None':
      self.cert = None
    self.key = self.get_parameter('key').value
    if self.key == 'None':
      self.key = None
    self.ca_cert = self.get_parameter('ca_cert').value
    if self.ca_cert == 'None':
      self.ca_cert = None

    self.rtcm_timeout_seconds = self.get_parameter('rtcm_timeout_seconds').value


  def init_ntrip_client(self):
    """Initialize a NTRIP client using class internal variable."""
    client = NTRIPClient(
      host=self.host,
      port=self.port,
      mountpoint=self.mountpoint,
      ntrip_version=self.ntrip_version,
      username=self.username,
      password=self.password,
      logerr=self.get_logger().error,
      logwarn=self.get_logger().warning,
      loginfo=self.get_logger().info,
      logdebug=self.get_logger().debug
      )
    client.ssl = self.ssl
    client.cert = self.cert
    client.key = self.key
    client.ca_cert = self.ca_cert

    client.nmea_parser.nmea_max_length = self._nmea_max_length
    client.nmea_parser.nmea_min_length = self._nmea_min_length
    client.reconnect_attempt_max = self._reconnect_attempt_max
    client.reconnect_attempt_wait_seconds = self._reconnect_attempt_wait_seconds
    client.rtcm_timeout_seconds = self.rtcm_timeout_seconds

    return client


if __name__ == '__main__':
  # Start the node
  rclpy.init()
  node = NTRIPRos()
  try:
    # Spin until we are shut down
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass

  # Shutdown the node and stop rclpy
  rclpy.shutdown()
