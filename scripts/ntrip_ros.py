#!/usr/bin/env python

import os
import sys
import json

import rospy
from std_msgs.msg import Header

try:
    from mavros_msgs.msg import RTCM
    HAVE_MAVROS = True
except:
    HAVE_MAVROS = False

try:
    from rtcm_msgs.msg import Message
    HAVE_RTCM = True
except:
    HAVE_RTCM = False

from nmea_msgs.msg import Sentence

from ntrip_client.ntrip_client import NTRIPClient


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

    self.message_type = rospy.get_param('~message_type', 'mavros_msgs')

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

    # Setup the RTCM publisher
    self._rtcm_timer = None
    if self.message_type == "mavros_msgs":
        if not HAVE_MAVROS:
            rospy.logerr("mavros_msgs package not found - please install ros-<distro>-mavros-msgs")
            return 1

        self._rtcm_pub = rospy.Publisher('rtcm', RTCM, queue_size=10)
    elif self.message_type == "rtcm_msgs":
        if not HAVE_RTCM:
            rospy.logerr("mavros_msgs package not found - please install ros-<distro>-rtcm-msgs")
            return 1

        self._rtcm_pub = rospy.Publisher('rtcm', Message, queue_size=10)
    else:
        rospy.logerr("[ntrip_client] Does not support message_type %s" % self.message_type)
        return 1

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

      if self.message_type == "mavros_msgs":
        rtcm_message = RTCM(
          header=Header(
            stamp=rospy.Time.now(),
            frame_id=self._rtcm_frame_id
          ),
          data=raw_rtcm
        )
      elif self.message_type == "rtcm_msgs":
        rtcm_message = Message(
          header=Header(
            stamp=rospy.Time.now(),
            frame_id=self._rtcm_frame_id
          ),
          message=raw_rtcm
        ) 
      self._rtcm_pub.publish(rtcm_message)


if __name__ == '__main__':
  ntrip_ros = NTRIPRos()
  sys.exit(ntrip_ros.run())
