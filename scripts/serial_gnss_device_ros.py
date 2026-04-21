#!/usr/bin/env python
import os
import sys
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from rtcm_msgs.msg import Message
from nmea_msgs.msg import Sentence

from ntrip_client.serial_gnss_device import SerialGNSSDevice
from ntrip_client.nmea_parser import NMEAParser, NMEA_DEFAULT_MAX_LENGTH, NMEA_DEFAULT_MIN_LENGTH
class SerialGNSSDeviceROS(Node):
  def __init__(self):
    # Read a debug flag from the environment that should have been set by the launch file
    try:
      self._debug = json.loads(os.environ["NTRIP_CLIENT_DEBUG"].lower())
    except:
      self._debug = False

    # Init the node and declare params
    super().__init__('serial_gnss_device')
    self.declare_parameters(
      namespace='',
      parameters=[
        ('nmea_frame_id', 'odom'),
        ('nmea_max_length', NMEA_DEFAULT_MAX_LENGTH),
        ('nmea_min_length', NMEA_DEFAULT_MIN_LENGTH),
        ('port', '/dev/ttyACM0'),
        ('baudrate', 115200),
      ]
    )

    if self._debug:
      rclpy.logging.set_logger_level(self.get_logger().name, rclpy.logging.LoggingSeverity.DEBUG)

    # Read some mandatory config
    port = self.get_parameter('port').value
    baudrate = self.get_parameter('baudrate').value

    self._gnss_device = SerialGNSSDevice(
      port=port, 
      baudrate=baudrate,   
      logerr=self.get_logger().error,
      logwarn=self.get_logger().warning,
      loginfo=self.get_logger().info,
      logdebug=self.get_logger().debug
    )

    # Read an optional Frame ID from the config
    self._nmea_frame_id = self.get_parameter('nmea_frame_id').value

    # Get NMEA length config
    self._gnss_device.nmea_parser.nmea_max_length = self.get_parameter('nmea_max_length').value
    self._gnss_device.nmea_parser.nmea_min_length = self.get_parameter('nmea_min_length').value

    # Setup NMEA publisher
    self._nmea_pub = self.create_publisher(Sentence, 'nmea', 10)

  def run(self):
    # Connect to GNSS device
    if not self._gnss_device.connect():
      self.get_logger().error('Unable to connect to GNSS device')
      return False

    # Setup our subscribers
    self._rtcm_sub = self.create_subscription(Message, 'rtcm', self.subscribe_rtcm, 10)

    # Start the timer that will check for NMEA data
    self._nmea_timer = self.create_timer(0.1, self.publish_nmea)
    return True
  
  def stop(self):
    self.get_logger().info('Stopping NMEA publisher')
    if self._nmea_timer:
      self._nmea_timer.cancel()
      self._nmea_timer.destroy()
    self._gnss_device.disconnect()
    self.get_logger().info('Shutting down node')
    self.destroy_node()
  
  def publish_nmea(self):
    for nmea_sentence in self._gnss_device.recv_nmea():
      self._nmea_pub.publish(self._create_nmea_sentence_nmea_packet(nmea_sentence))

  def _create_nmea_sentence_nmea_packet(self, nmea_sentence):
    return Sentence(
      header=Header(
        stamp = self.get_clock().now().to_msg(),
        frame_id = self._nmea_frame_id
      ),
      sentence = nmea_sentence
    )
  
  def subscribe_rtcm(self, rtcm):
    # Extract RTCM data from the mesage and send it to the gnss_device
    self._gnss_device.send_rtmc(rtcm.message)

if __name__ == '__main__':
  # Start the node
  rclpy.init()
  node = SerialGNSSDeviceROS()
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