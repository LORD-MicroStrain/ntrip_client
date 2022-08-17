#!/usr/bin/env python

import os
import sys
import json
import importlib

import rospy
from std_msgs.msg import Header
from nmea_msgs.msg import Sentence

from ntrip_client.ntrip_client import NTRIPClient
from ntrip_client.nmea_parser import NMEA_DEFAULT_MAX_LENGTH, NMEA_DEFAULT_MIN_LENGTH

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

class NTRIPRos:
  def __init__(self):
    # Read a debug flag from the environment that should have been set by the launch file
    try:
      self._debug = json.loads(os.environ["NTRIP_CLIENT_DEBUG"].lower())
    except:
      self._debug = False

    # Init the node and read some mandatory config
    if self._debug:
      rospy.init_node('ntrip_client', anonymous=True, log_level=rospy.DEBUG)
    else:
      rospy.init_node('ntrip_client', anonymous=True)
    host = rospy.get_param('~host', '127.0.0.1')
    port = rospy.get_param('~port', '2101')
    mountpoint = rospy.get_param('~mountpoint', 'mount')

    # Optionally get the ntrip version from the launch file
    ntrip_version = rospy.get_param('~ntrip_version', None)
    if ntrip_version == '':
      ntrip_version = None

    # If we were asked to authenticate, read the username and password
    username = None
    password = None
    if rospy.get_param('~authenticate', False):
      username = rospy.get_param('~username', None)
      password = rospy.get_param('~password', None)
      if username is None:
        rospy.logerr(
          'Requested to authenticate, but param "username" was not set')
        sys.exit(1)
      if password is None:
        rospy.logerr(
          'Requested to authenticate, but param "password" was not set')
        sys.exit(1)

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
    self._client = NTRIPClient(
      host=host,
      port=port,
      mountpoint=mountpoint,
      ntrip_version=ntrip_version,
      username=username,
      password=password,
      logerr=rospy.logerr,
      logwarn=rospy.logwarn,
      loginfo=rospy.loginfo,
      logdebug=rospy.logdebug
    )

    # Get some SSL parameters for the NTRIP client
    self._client.ssl = rospy.get_param('~ssl', False)
    self._client.cert = rospy.get_param('~cert', None)
    self._client.key = rospy.get_param('~key', None)
    self._client.ca_cert = rospy.get_param('~ca_cert', None)

    # Set parameters on the client
    self._client.nmea_parser.nmea_max_length = rospy.get_param('~nmea_max_length', NMEA_DEFAULT_MAX_LENGTH)
    self._client.nmea_parser.nmea_min_length = rospy.get_param('~nmea_min_length', NMEA_DEFAULT_MIN_LENGTH)
    self._client.reconnect_attempt_max = rospy.get_param('~reconnect_attempt_max', NTRIPClient.DEFAULT_RECONNECT_ATTEMPT_MAX)
    self._client.reconnect_attempt_wait_seconds = rospy.get_param('~reconnect_attempt_wait_seconds', NTRIPClient.DEFAULT_RECONNECT_ATEMPT_WAIT_SECONDS)
    self._client.rtcm_timeout_seconds = rospy.get_param('~rtcm_timeout_seconds', NTRIPClient.DEFAULT_RTCM_TIMEOUT_SECONDS)

  def run(self):
    # Setup a shutdown hook
    rospy.on_shutdown(self.stop)

    # Connect the client
    if not self._client.connect():
      rospy.logerr('Unable to connect to NTRIP server')
      return 1

    # Setup our subscriber
    self._nmea_sub = rospy.Subscriber('nmea', Sentence, self.subscribe_nmea, queue_size=10)

    # Start the timer that will check for RTCM data
    self._rtcm_timer = rospy.Timer(rospy.Duration(0.1), self.publish_rtcm)

    # Spin until we are shutdown
    rospy.spin()
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


if __name__ == '__main__':
  ntrip_ros = NTRIPRos()
  sys.exit(ntrip_ros.run())
