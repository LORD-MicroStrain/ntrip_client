#!/usr/bin/env python3

import sys
import importlib

import rospy

from ntrip_client.ntrip_ros_base import NTRIPRosBase
from ntrip_client.ntrip_serial_device import NTRIPSerialDevice

# Try to import a couple different types of RTCM messages
_MAVROS_MSGS_NAME = "mavros_msgs"
_RTCM_MSGS_NAME = "rtcm_msgs"
have_mavros_msgs = False
have_rtcm_msgs = False
if importlib.util.find_spec(_MAVROS_MSGS_NAME) is not None:
  have_mavros_msgs = True
  from mavros_msgs.msg import RTCM as mavros_msgs_RTCM
if importlib.util.find_spec(_RTCM_MSGS_NAME) is not None:
  have_rtcm_msgs = True
  from rtcm_msgs.msg import Message as rtcm_msgs_RTCM

class NTRIPRos(NTRIPRosBase):
  def __init__(self):
    # Init the node
    super().__init__('ntrip_client')

    # Read some mandatory config
    port = rospy.get_param('~port', '/dev/ttyACM0')
    baudrate = rospy.get_param('~baudrate', 115200)

    # Initialize the client
    self._client = NTRIPSerialDevice(
      port=port,
      baudrate=baudrate,
      logerr=rospy.logerr,
      logwarn=rospy.logwarn,
      loginfo=rospy.loginfo,
      logdebug=rospy.logdebug
    )

    # Set parameters on the client
    self._client.nmea_parser.nmea_max_length = self._nmea_max_length
    self._client.nmea_parser.nmea_min_length = self._nmea_min_length
    self._client.reconnect_attempt_max = self._reconnect_attempt_max
    self._client.reconnect_attempt_wait_seconds = self._reconnect_attempt_wait_seconds
    self._client.reconnect_backoff_base = self._reconnect_backoff_base
    self._client.reconnect_backoff_max_seconds = self._reconnect_backoff_max_seconds


if __name__ == '__main__':
  ntrip_ros = NTRIPRos()
  sys.exit(ntrip_ros.run())
