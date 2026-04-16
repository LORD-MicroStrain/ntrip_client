import time
import logging

from .nmea_parser import NMEAParser
from .rtcm_parser import RTCMParser

class NTRIPBase:

  # Public constants
  DEFAULT_RECONNECT_ATTEMPT_MAX = 10
  DEFAULT_RECONNECT_ATEMPT_WAIT_SECONDS = 5

  def __init__(self, logerr=logging.error, logwarn=logging.warning, loginfo=logging.info, logdebug=logging.debug):
    # Bit of a strange pattern here, but save the log functions so we can be agnostic of ROS
    self._logerr = logerr
    self._logwarn = logwarn
    self._loginfo = loginfo
    self._logdebug = logdebug

    # Setup some parsers to parse incoming messages
    self.rtcm_parser = RTCMParser(
      logerr=logerr,
      logwarn=logwarn,
      loginfo=loginfo,
      logdebug=logdebug
    )
    self.nmea_parser = NMEAParser(
      logerr=logerr,
      logwarn=logwarn,
      loginfo=loginfo,
      logdebug=logdebug
    )

    # Setup some state
    self._shutdown = False
    self._connected = False

    # Public reconnect info
    self.reconnect_attempt_max = self.DEFAULT_RECONNECT_ATTEMPT_MAX
    self.reconnect_attempt_wait_seconds = self.DEFAULT_RECONNECT_ATEMPT_WAIT_SECONDS

  def connect(self):
    raise NotImplementedError("Must override connect")
  
  def disconnect(self):
    raise NotImplementedError("Must override disconnect")

  def reconnect(self):
    self.disconnect()
    while not self._shutdown:
      self._reconnect_attempt_count += 1
      connect_success = self.connect()
      if connect_success:
        self._reconnect_attempt_count = 0
        break
      self._logerr('Reconnect attempt {} failed. Retrying in {} seconds'.format(
        self._reconnect_attempt_count, self.reconnect_attempt_wait_seconds))
      time.sleep(self.reconnect_attempt_wait_seconds)

  def send_nmea(self):
    raise NotImplementedError("Must override send_nmea")

  def recv_rtcm(self):
    raise NotImplementedError("Must override recv_rtcm")

  def shutdown(self):
    # Set some state, and then disconnect
    self._shutdown = True
    self.disconnect()
