import time
import logging

from .nmea_parser import NMEAParser
from .rtcm_parser import RTCMParser

class NTRIPBase:

  # Public constants
  DEFAULT_RECONNECT_ATTEMPT_MAX = 10
  DEFAULT_RECONNECT_ATEMPT_WAIT_SECONDS = 5

  DEFAULT_RECONNECT_BACKOFF_BASE = 1.8
  DEFAULT_RECONNECT_BACKOFF_MAX_SECONDS = 300

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
    # How many connection attempts have failed since we last connected?
    # We don't consider connection successful until some valid data has been received.
    # TODO merge _reconnect_attempts into this, since it seems to track almost the same
    self._failed_connections = 0

    # Public reconnect info
    self.reconnect_attempt_max = self.DEFAULT_RECONNECT_ATTEMPT_MAX
    self.reconnect_attempt_wait_seconds = self.DEFAULT_RECONNECT_ATEMPT_WAIT_SECONDS
    self.reconnect_backoff_base = self.DEFAULT_RECONNECT_BACKOFF_BASE
    self.reconnect_backoff_max_seconds = self.DEFAULT_RECONNECT_BACKOFF_MAX_SECONDS

  def connect(self):
    raise NotImplementedError("Must override connect")
  
  def disconnect(self):
    raise NotImplementedError("Must override disconnect")

  def _compute_reconnect_wait_time(self):
    """
    Compute a time to sleep before attempting to reconnect.

    This is based on an exponential backoff, capped to a maximum.

    All of the initial wait times, the maximum and the base are configurable.
    """
    return min(
      self.reconnect_attempt_wait_seconds * (self.reconnect_backoff_base ** self._failed_connections),
      self.reconnect_backoff_max_seconds
    )

  def reconnect(self, initial = False):
    if self._connected or initial:
      while not self._shutdown:
        self._reconnect_attempt_count += 1
        if not initial:
          self.disconnect()
          to_wait = self._compute_reconnect_wait_time()
          self._logerr(f"Reconnecting in {to_wait} seconds")
          time.sleep(self._compute_reconnect_wait_time())
        initial = False
        self._failed_connections += 1
        connect_success = self.connect()
        if not connect_success and self._reconnect_attempt_count < self.reconnect_attempt_max:
          self._logerr('Reconnect failed')
        elif self._reconnect_attempt_count >= self.reconnect_attempt_max:
          self._reconnect_attempt_count = 0
          raise Exception("Reconnect was attempted {} times, but never succeeded".format(self._reconnect_attempt_count))
        elif connect_success:
          self._reconnect_attempt_count = 0
          break
    else:
      self._logdebug('Reconnect called while not connected, ignoring')

  def mark_successful_connection(self):
    self._failed_connections = 0

  def send_nmea(self):
    raise NotImplementedError("Must override send_nmea")

  def recv_rtcm(self):
    raise NotImplementedError("Must override recv_rtcm")

  def shutdown(self):
    # Set some state, and then disconnect
    self._shutdown = True
    self.disconnect()
