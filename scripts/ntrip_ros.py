#!/usr/bin/env python

import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from mavros_msgs.msg import RTCM
from nmea_msgs.msg import Sentence

from ntrip_client.ntrip_client import NTRIPClient


class NTRIPRos(Node):
  def __init__(self):
    # Init the node and declare params
    super().__init__('ntrip_client')
    self.declare_parameters(
      namespace='',
      parameters=[
        ('host', '127.0.0.1'),
        ('port', 2101),
        ('mountpoint', 'mount'),
        ('authenticate', False),
        ('username', ''),
        ('password', ''),
        ('rtcm_frame_id', 'odom')
      ]
    )

    # Read some mandatory config
    host = self.get_parameter('host').value
    port = self.get_parameter('port').value
    mountpoint = self.get_parameter('mountpoint').value

    # If we were asked to authenticate, read the username and password
    username = None
    password = None
    if self.get_parameter('authenticate').value:
      username = self.get_parameter('username').value
      password = self.get_parameter('password').value
      if not username:
        self.get_logger().error(
          'Requested to authenticate, but param "username" was not set')
        sys.exit(1)
      if not password:
        self.get_logger().error(
          'Requested to authenticate, but param "password" was not set')
        sys.exit(1)

    # Read an optional Frame ID from the config
    self._rtcm_frame_id = self.get_parameter('rtcm_frame_id').value

    # Setup the RTCM publisher
    self._rtcm_pub = self.create_publisher(RTCM, 'rtcm', 10)

    # Initialize the client
    self._client = NTRIPClient(
      host=host,
      port=port,
      mountpoint=mountpoint,
      username=username,
      password=password,
      logerr=self.get_logger().error,
      logwarn=self.get_logger().warning,
      loginfo=self.get_logger().info,
      logdebug=self.get_logger().debug
    )

  def run(self):
    # Connect the client
    if not self._client.connect():
      self.get_logger().error('Unable to connect to NTRIP server')
      return False
    # Setup our subscriber
    self._nmea_sub = self.create_subscription(Sentence, 'nmea', self.subscribe_nmea, 10)

    # Start the timer that will check for RTCM data
    self._rtcm_timer = self.create_timer(0.1, self.publish_rtcm)
    return True

  def stop(self):
    self.get_logger().info('Stopping RTCM publisher')
    if self._rtcm_timer:
      self._rtcm_timer.cancel()
      self._rtcm_timer.destroy()
    self.get_logger().info('Disconnecting NTRIP client')
    self._client.disconnect()
    self.get_logger().info('Shutting down node')
    self.destroy_node()

  def subscribe_nmea(self, nmea):
    # Just extract the NMEA from the message, and send it right to the server
    self._client.send_nmea(nmea.sentence)

  def publish_rtcm(self):
    for raw_rtcm in self._client.recv_rtcm():
      self._rtcm_pub.publish(RTCM(
        header=Header(
          stamp=self.get_clock().now().to_msg(),
          frame_id=self._rtcm_frame_id
        ),
        data=raw_rtcm
      ))


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