#!/usr/bin/env python

import os
import sys
import json

import rospy
from std_msgs.msg import Header

from nmea_msgs.msg import Sentence

from ntrip_client.ntrip_client import NTRIPClient

# Try to import rtcm msg from mavros_msgs
have_mavros_msgs = False
try:
  from mavros_msgs.msg import RTCM
  have_mavros_msgs = True
  print('mavros_msgs')
except:
  print('Unable to import from mavros_msgs')

# try to import rtcm msg from rtcm_msgs
have_rtcm_msgs = False
try:
  from rtcm_msgs.msg import Message
  have_rtcm_msgs = True
  print('rtcm_msgs')
except:
  print('Unable to import from rtcm_msgs')

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
    self.rtcm_msg_pkg = rospy.get_param('~rtcm_msg_pkg', 'mavros_msgs')

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

    # Setup the RTCM publisher
    self._rtcm_timer = None
    if self.rtcm_msg_pkg == 'mavros_msgs' and have_mavros_msgs == True:
      self._rtcm_pub = rospy.Publisher('rtcm', RTCM, queue_size=10)
    elif self.rtcm_msg_pkg == 'rtcm_msgs' and have_rtcm_msgs == True:
      self._rtcm_pub = rospy.Publisher('rtcm', Message, queue_size=10)
    else:
      rospy.logfatal('The rtcm message package {} could not be imported - check it is installed'.format(self.rtcm_msg_pkg))
      raise Exception('The rtcm message package {} could not be imported - check it is installed'.format(self.rtcm_msg_pkg))

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
    self._client.disconnect()

  def subscribe_nmea(self, nmea):
    # Just extract the NMEA from the message, and send it right to the server
    self._client.send_nmea(nmea.sentence)

  def publish_rtcm(self, event):
    try:
      for raw_rtcm in self._client.recv_rtcm():
        if self.rtcm_msg_pkg == 'mavros_msgs':
          rtcm_msg = RTCM(
            header=Header(
              stamp=rospy.Time.now(),
              frame_id=self._rtcm_frame_id
            ),
            data=raw_rtcm
          )
        elif self.rtcm_msg_pkg == 'rtcm_msgs':
          rtcm_msg = Message(
            header=Header(
              stamp=rospy.Time.now(),
              frame_id=self._rtcm_frame_id
            ),
            message =raw_rtcm
          )
        self._rtcm_pub.publish(rtcm_msg)
    except:
      rospy.logerr('RTCM publish to topic failed')

if __name__ == '__main__':
  ntrip_ros = NTRIPRos()
  sys.exit(ntrip_ros.run())
