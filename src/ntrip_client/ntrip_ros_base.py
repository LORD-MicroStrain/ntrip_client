#!/usr/bin/env python

import os
import json
import datetime
import importlib

import rospy
from std_msgs.msg import Header
from nmea_msgs.msg import Sentence
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus

from ntrip_client.ntrip_base import NTRIPBase
from ntrip_client.nmea_parser import NMEAParser, NMEA_DEFAULT_MAX_LENGTH, NMEA_DEFAULT_MIN_LENGTH

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

class NTRIPRosBase:
  def __init__(self, name):
    # Read a debug flag from the environment that should have been set by the launch file
    try:
      self._debug = json.loads(os.environ["NTRIP_CLIENT_DEBUG"].lower())
    except:
      self._debug = False

    # Init the node and read some mandatory config
    if self._debug:
      rospy.init_node(name, anonymous=True, log_level=rospy.DEBUG)
    else:
      rospy.init_node(name, anonymous=True)

    # Read an optional Frame ID from the config
    self._rtcm_frame_id = rospy.get_param('~rtcm_frame_id', 'odom')

    # Determine the type of RTCM message that will be published
    rtcm_message_package = rospy.get_param('~rtcm_message_package', _MAVROS_MSGS_NAME)
    if rtcm_message_package == _MAVROS_MSGS_NAME:
      if have_mavros_msgs:
        self._rtcm_message_type = mavros_msgs_RTCM
        self._create_rtcm_message = self._create_mavros_msgs_rtcm_message
      else:
        rospy.logfatal('The requested RTCM package {} is a valid option, but we were unable to import it. Please make sure you have it installed'.format(rtcm_message_package))
    elif rtcm_message_package == _RTCM_MSGS_NAME:
      if have_rtcm_msgs:
        self._rtcm_message_type = rtcm_msgs_RTCM
        self._create_rtcm_message = self._create_rtcm_msgs_rtcm_message
      else:
        rospy.logfatal('The requested RTCM package {} is a valid option, but we were unable to import it. Please make sure you have it installed'.format(rtcm_message_package))
    else:
      rospy.logfatal('The RTCM package {} is not a valid option. Please choose between the following packages {}'.format(rtcm_message_package, str.join([_MAVROS_MSGS_NAME, _RTCM_MSGS_NAME])))

    # Setup the RTCM publisher
    self._rtcm_timer = None
    self._rtcm_pub = rospy.Publisher('rtcm', self._rtcm_message_type, queue_size=10)

    # Initialize the client
    self._client = NTRIPBase(
      logerr=rospy.logerr,
      logwarn=rospy.logwarn,
      loginfo=rospy.loginfo,
      logdebug=rospy.logdebug
    )

    # Set parameters on the client
    self._nmea_max_length = rospy.get_param('~nmea_max_length', NMEA_DEFAULT_MAX_LENGTH)
    self._nmea_min_length = rospy.get_param('~nmea_min_length', NMEA_DEFAULT_MIN_LENGTH)
    self._reconnect_attempt_max = rospy.get_param('~reconnect_attempt_max', NTRIPBase.DEFAULT_RECONNECT_ATTEMPT_MAX)
    self._reconnect_attempt_wait_seconds = rospy.get_param('~reconnect_attempt_wait_seconds', NTRIPBase.DEFAULT_RECONNECT_ATEMPT_WAIT_SECONDS)

  def run(self):
    # Setup a shutdown hook
    rospy.on_shutdown(self.stop)

    # Connect the client
    if not self._client.connect():
      rospy.logerr('Unable to connect to NTRIP server')
      return 1

    # Setup our subscriber
    self._nmea_sub = rospy.Subscriber('nmea', Sentence, self.subscribe_nmea, queue_size=10)
    self._fix_sub = rospy.Subscriber('fix', NavSatFix, self.subscribe_fix, queue_size=10)

    # Start the timer that will check for RTCM data
    self._rtcm_timer = rospy.Timer(rospy.Duration(0.1), self.publish_rtcm)

    # Spin until we are shutdown
    while not self._client._shutdown:
      rospy.sleep(1)
    return 0

  def stop(self):
    rospy.loginfo('Stopping RTCM publisher')
    if self._rtcm_timer:
      self._rtcm_timer.shutdown()
      self._rtcm_timer.join()
    rospy.loginfo('Disconnecting NTRIP client')
    self._client.shutdown()

  def subscribe_nmea(self, nmea):
    # Just extract the NMEA from the message, and send it right to the server
    self._client.send_nmea(nmea.sentence)

  def subscribe_fix(self, fix: NavSatFix):
    # Calculate the timestamp of the message
    timestamp_secs = fix.header.stamp.secs + fix.header.stamp.nsecs * 1e-9
    timestamp = datetime.datetime.fromtimestamp(timestamp_secs)
    time = timestamp.time()
    hour = time.hour
    minute = time.minute
    second = time.second
    millisecond = int(time.microsecond * 1e-4)
    nmea_utc = f"{hour:02}{minute:02}{second:02}.{millisecond:02}"

    # Figure out the direction of the latitude and longitude
    nmea_lat_direction = "N"
    nmea_lon_direction = "E"
    if fix.latitude < 0:
      nmea_lat_direction = "S"
    if fix.longitude < 0:
      nmea_lon_direction = "W"

    # Convert the units of the latitude and longitude
    nmea_lat = NMEAParser.lat_dd_to_dmm(fix.latitude)
    nmea_lon = NMEAParser.lon_dd_to_dmm(fix.longitude)

    # Convert the GPS quality to the right format for the sentence
    status = fix.status.status
    if status == NavSatStatus.STATUS_FIX:
      nmea_status = 1
    elif status == NavSatStatus.STATUS_SBAS_FIX:
      nmea_status = 2
    elif status == NavSatStatus.STATUS_GBAS_FIX:
      nmea_status = 5
    else:
      nmea_status = 0

    # Assemble the sentence
    nmea_sentence_no_checksum = f"$GPGGA,{nmea_utc},{nmea_lat},{nmea_lat_direction},{nmea_lon},{nmea_lon_direction},{nmea_status},05,1.0,100.0,M,-32.0,M,,0000"
    nmea_checksum = NMEAParser.checksum(nmea_sentence_no_checksum)
    nmea_sentence = f"{nmea_sentence_no_checksum}*{nmea_checksum:x}\r\n"

    # Send the sentence to the client
    self._client.send_nmea(nmea_sentence)


  def publish_rtcm(self, event):
    for raw_rtcm in self._client.recv_rtcm():
      self._rtcm_pub.publish(self._create_rtcm_message(raw_rtcm))

  def _create_mavros_msgs_rtcm_message(self, rtcm):
    return mavros_msgs_RTCM(
      header=Header(
        stamp=rospy.Time.now(),
        frame_id=self._rtcm_frame_id
      ),
      data=rtcm
    )

  def _create_rtcm_msgs_rtcm_message(self, rtcm):
    return rtcm_msgs_RTCM(
      header=Header(
        stamp=rospy.Time.now(),
        frame_id=self._rtcm_frame_id
      ),
      message=rtcm
    )
