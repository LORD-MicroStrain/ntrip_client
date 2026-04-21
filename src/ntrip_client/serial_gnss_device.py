#!/usr/bin/evn python

import time
import logging
import serial

from .nmea_parser import NMEAParser
from .rtcm_parser import RTCMParser

class SerialGNSSDevice:

  # Public constants
  DEFAULT_RECONNECT_ATTEMPT_MAX = 10
  DEFAULT_RECONNECT_ATEMPT_WAIT_SECONDS = 5

  def __init__(self, port, baudrate, logerr = logging.error, logwarn=logging.warning, loginfo=logging.info, logdebug=logging.debug):
        
    # Bit of a strange pattern here, but save the log functions so we can be agnostic of ROS
    self._reconnect_attempt_count = 0
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

    # Save the connection info
    self._port = port
    self._baudrate = baudrate

    # Initialize this so we don't throw an exception when closing
    self._device = None

  def connect(self):
    # Attempt to open the serial port
    try:
      self._device = serial.Serial(self._port, self._baudrate)
    except Exception as e:
      self._logerr('Unable to open serial port {} at baudrate {}'.format(self._port, self._baudrate))
      self._logerr('Exception: {}'.format(str(e)))
      return False
    
    # Right now, we can't check anything else, so assuming that the port is open, we succeeded.
    self._loginfo('Connected to serial port {} at baudrate {}'.format(self._port, self._baudrate))
    self._connected = True
    return True

  def disconnect(self):
    # Disconnect the serial port
    try:
      if self._device:
        self._device.close()
    except Exception as e:
      self._logdebug('Encountered exception when closing the serial port. This can likely be ignored.')
      self._logdebug('Exception: {}'.format(str(e)))
  
  def reconnect(self):
    if self._connected:
      while not self._shutdown:
        self._reconnect_attempt_count += 1
        self.disconnect()
        connect_success = self.connect()
        if not connect_success and self._reconnect_attempt_count < self.reconnect_attempt_max:
          self._logerr('Reconnect failed. Retrying in {} seconds'.format(self.reconnect_attempt_wait_seconds)) 
          time.sleep(self.reconnect_attempt_wait_seconds)
        elif self._reconnect_attempt_count >= self.reconnect_attempt_max:
          self._reconnect_attempt_count = 0
          raise Exception("Reconnect was attempted {} times, but never succeeded".format(self._reconnect_attempt_count))
        elif connect_success:
          self._reconnect_attempt_count = 0
          break
    else:
      self._logdebug('Reconnect called while not connected, ignoring')

  def recv_nmea(self):
    if not self._connected:
      self._logwarn('NMEA received before port was connected, returning empty string')
      return []
    # Check how much data is available on the device
    if self._device.in_waiting:
      try: 
        data = self._device.read_all()
        self._logdebug('Read {} bytes from serial device'.format(len(data)))
        return self.nmea_parser.parse(data)
      except Exception as e:
        self._logerr('Unable to read from serial device, reconnecting...')
        self.reconnect()
        return []
    else:
      return []
              
  def send_rtmc(self, packet):
    if not self._connected:
      self._logwarn('RTCM sent before device was connected, returing empty list')
      return []
    
    # Check if RTCM packet is valid
    if not self.rtcm_parser.is_valid_packet(packet):
      self._logwarn('Invalid RTCM packet, not sending to GNSS device')
      return
    
    try:
      self._device.write(packet)
    except Exception as e:
      self._logerr('Unable to send RTCM packet to device, check COM port settings.')
    
