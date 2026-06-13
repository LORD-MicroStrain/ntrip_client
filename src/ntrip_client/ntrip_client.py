#!/usr/bin/env python3

import ssl
import time
import base64
import socket
import select
import logging

from .ntrip_base import NTRIPBase

_CHUNK_SIZE = 1024
_SOURCETABLE_RESPONSES = [
  'SOURCETABLE 200 OK'
]
_SUCCESS_RESPONSES = [
  'ICY 200 OK',
  'HTTP/1.0 200 OK',
  'HTTP/1.1 200 OK'
]
_UNAUTHORIZED_RESPONSES = [
  '401'
]

class NTRIPClient(NTRIPBase):

  # Public constants
  DEFAULT_RECONNECT_ATEMPT_WAIT_SECONDS = 5
  DEFAULT_RECONNECT_ATTEMPT_WAIT_MAX_SECONDS = 120
  DEFAULT_RTCM_TIMEOUT_SECONDS = 10

  # Default User-Agent. NOTE: rtk2go.com (SNIP) blocks the stock LORD Microstrain
  # signature 'NTRIP ntrip_client_ros' and refuses such clients by returning the
  # sourcetable instead of the stream. Per NTRIP it must begin with 'NTRIP '. Use a
  # unique string that identifies this robot so we don't share a blocked signature.
  DEFAULT_USER_AGENT = 'NTRIP ros_ntrip_client'

  def __init__(self, host, port, mountpoint, ntrip_version, username, password, user_agent=DEFAULT_USER_AGENT, logerr=logging.error, logwarn=logging.warning, loginfo=logging.info, logdebug=logging.debug):
    # Call the parent constructor
    super().__init__(logerr, logwarn, loginfo, logdebug)

    # Save the server info
    self._host = host
    self._port = port
    self._mountpoint = mountpoint
    self._ntrip_version = ntrip_version
    self._user_agent = user_agent if user_agent else self.DEFAULT_USER_AGENT
    if username is not None and password is not None:
      self._basic_credentials = base64.b64encode('{}:{}'.format(
        username, password).encode('utf-8')).decode('utf-8')
    else:
      self._basic_credentials = None

    # Initialize this so we don't throw an exception when closing
    self._raw_socket = None
    self._server_socket = None

    # Public SSL configuration
    self.ssl = False
    self.cert = None
    self.key = None
    self.ca_cert = None

    # Setup some state
    self._shutdown = False
    self._connected = False

    # Private reconnect info
    self._reconnect_attempt_count = 0
    self._nmea_send_failed_count = 0
    self._nmea_send_failed_max = 5
    self._read_zero_bytes_count = 0
    self._read_zero_bytes_max = 5
    self._first_rtcm_received = False
    self._recv_rtcm_last_packet_timestamp = 0

    # Reconnect scheduling (non-blocking)
    self._reconnect_pending = False
    self._reconnect_next_time = 0
    self._current_backoff = 0

    # Public reconnect info
    self.reconnect_attempt_wait_seconds = self.DEFAULT_RECONNECT_ATEMPT_WAIT_SECONDS
    self.reconnect_attempt_wait_max_seconds = self.DEFAULT_RECONNECT_ATTEMPT_WAIT_MAX_SECONDS
    self.rtcm_timeout_seconds = self.DEFAULT_RTCM_TIMEOUT_SECONDS

  def connect(self):
    # Create a socket object that we will use to connect to the server
    self._server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    self._server_socket.settimeout(5)

    # Connect the socket to the server
    try:
      self._server_socket.connect((self._host, self._port))
    except Exception as e:
      self._logerr('Unable to connect socket to server at http://{}:{}'.format(self._host, self._port))
      self._logerr('Exception: {}'.format(str(e)))
      return False

    # If SSL, wrap the socket
    if self.ssl:
      # Configre the context based on the config
      self._ssl_context = ssl.create_default_context()
      if self.cert:
        self._ssl_context.load_cert_chain(self.cert, self.key)
      if self.ca_cert:
        self._ssl_context.load_verify_locations(self.ca_cert)

      # Save the old socket for later just in case, and create a new SSL socket
      self._raw_socket = self._server_socket
      self._server_socket = self._ssl_context.wrap_socket(self._raw_socket, server_hostname=self._host)

    # Send the HTTP Request
    try:
      self._server_socket.send(self._form_request())
    except Exception as e:
      self._logerr('Unable to send request to server at http://{}:{}'.format(self._host, self._port))
      self._logerr('Exception: {}'.format(str(e)))
      return False

    # Get the response from the server
    response = ''
    try:
      response = self._server_socket.recv(_CHUNK_SIZE).decode('ISO-8859-1')
    except Exception as e:
      self._logerr('Unable to read response from server at http://{}:{}'.format(self._host, self._port))
      self._logerr('Exception: {}'.format(str(e)))
      return False

    # Properly handle the response
    if any(success in response for success in _SUCCESS_RESPONSES):
      self._connected = True

    # Some debugging hints about the kind of error we received
    known_error = False
    if any(sourcetable in response for sourcetable in _SOURCETABLE_RESPONSES):
      self._logwarn('Received sourcetable response from the server instead of the stream. This means the caster refused the stream request: either the mountpoint is not valid, or the caster is blocking this client (rtk2go blocks the stock "NTRIP ntrip_client_ros" User-Agent). Current User-Agent: {}'.format(self._user_agent))
      known_error = True
    elif any(unauthorized in response for unauthorized in _UNAUTHORIZED_RESPONSES):
      self._logwarn('Received unauthorized response from the server. Check your username, password, and mountpoint to make sure they are correct.')
      known_error = True
    elif not self._connected:  # and (self._ntrip_version == None or self._ntrip_version == ''):
      self._logwarn(response)
      self._logwarn('Received unknown error from the server. Note that the NTRIP version was not specified in the launch file. This is not necesarilly the cause of this error, but it may be worth checking your NTRIP casters documentation to see if the NTRIP version needs to be specified.')
      known_error = True

    # Wish we could just return from the above checks, but some casters return both a success and an error in the response
    # If we received any known error, even if we received a success it should be considered a failure
    if known_error or not self._connected:
      self._logerr('Invalid response received from http://{}:{}/{}'.format(
        self._host, self._port, self._mountpoint))
      self._logerr('Response: {}'.format(response))
      return False
    else:
      self._loginfo(
        'Connected to http://{}:{}/{}'.format(self._host, self._port, self._mountpoint))
      return True


  def disconnect(self):
    # Disconnect the socket
    self._connected = False
    try:
      if self._server_socket:
        self._server_socket.shutdown(socket.SHUT_RDWR)
      if self._raw_socket:
        self._raw_socket.shutdown(socket.SHUT_RDWR)
    except Exception as e:
      self._logdebug('Encountered exception when shutting down the socket. This can likely be ignored')
      self._logdebug('Exception: {}'.format(e))
    try:
      if self._server_socket:
        self._server_socket.close()
      if self._raw_socket:
        self._raw_socket.close()
    except Exception as e:
      self._logdebug('Encountered exception when closing the socket. This can likely be ignored')
      self._logdebug('Exception: {}'.format(e))

  def request_reconnect(self, reason='Connection lost'):
    """Schedule a non-blocking reconnect. The actual attempt happens in try_reconnect()."""
    if self._reconnect_pending:
      return
    self.disconnect()
    self._reconnect_pending = True
    self._reconnect_attempt_count = 0
    self._current_backoff = self.reconnect_attempt_wait_seconds
    self._reconnect_next_time = time.time() + self._current_backoff
    self._logwarn('{}. Will retry in {} seconds'.format(reason, self._current_backoff))

  def try_reconnect(self):
    """Attempt one reconnect if the backoff timer has elapsed. Returns True if connected."""
    if not self._reconnect_pending:
      return self._connected

    now = time.time()
    if now < self._reconnect_next_time:
      return False

    self._reconnect_attempt_count += 1
    connect_success = self.connect()
    if connect_success:
      self._loginfo('Reconnected after {} attempts'.format(self._reconnect_attempt_count))
      self._reconnect_pending = False
      self._reconnect_attempt_count = 0
      self._first_rtcm_received = False
      return True

    # Exponential backoff: double the wait, capped at max
    self._current_backoff = min(self._current_backoff * 2, self.reconnect_attempt_wait_max_seconds)
    self._reconnect_next_time = now + self._current_backoff
    self._logerr('Reconnect attempt {} to http://{}:{} failed. Retrying in {} seconds'.format(
      self._reconnect_attempt_count, self._host, self._port, self._current_backoff))
    return False

  @property
  def reconnecting(self):
    return self._reconnect_pending

  def send_nmea(self, sentence):
    if not self._connected:
      self._logwarn('NMEA sent before client was connected, discarding NMEA')
      return

    # Not sure if this is the right thing to do, but python will escape the return characters at the end of the string, so do this manually
    if sentence[-4:] == '\\r\\n':
      sentence = sentence[:-4] + '\r\n'
    elif sentence[-2:] != '\r\n':
      sentence = sentence + '\r\n'

    # Check if it is a valid NMEA sentence
    if not self.nmea_parser.is_valid_sentence(sentence):
      self._logwarn("Invalid NMEA sentence, not sending to server")
      return

    # Encode the data and send it to the socket
    try:
      self._server_socket.send(sentence.encode('utf-8'))
    except Exception as e:
      self._logwarn('Unable to send NMEA sentence to server.')
      self._logwarn('Exception: {}'.format(str(e)))
      self._nmea_send_failed_count += 1
      if self._nmea_send_failed_count >= self._nmea_send_failed_max:
        self._logwarn("NMEA sentence failed to send to server {} times, reconnecting".format(self._nmea_send_failed_count))
        self.request_reconnect()
        self._nmea_send_failed_count = 0


  def recv_rtcm(self):
    # If a reconnect is in progress, try it and return empty until connected
    if self._reconnect_pending:
      self.try_reconnect()
      return []

    if not self._connected:
      self._logwarn('RTCM requested before client was connected, returning empty list')
      return []

    # If it has been too long since we received an RTCM packet, reconnect.
    # KNOWN LIMITATION: this watchdog only arms after the first RTCM packet ever
    # arrives (_first_rtcm_received). If the caster accepts the connection
    # (_connected=True) but never delivers any RTCM -- e.g. a mountpoint that is
    # down for maintenance while the caster still completes the GET -- this never
    # fires, the node believes it is connected, and (when send_nmea is true) keeps
    # uploading NMEA every cycle to a dead stream. On rtk2go that can earn a ban.
    # For fixed-base mountpoints the correct fix is send_nmea:=false (no NMEA at
    # all). VRS mountpoints require NMEA and would need this watchdog to instead
    # baseline off the connect time; left as documented behavior pending a VRS to
    # test against. See ntrip_client README "rtk2go and reconnect behavior".
    if time.time() - self.rtcm_timeout_seconds >= self._recv_rtcm_last_packet_timestamp and self._first_rtcm_received:
      self._logerr('RTCM data not received for {} seconds, reconnecting'.format(self.rtcm_timeout_seconds))
      self.request_reconnect()
      return []

    # Check if there is any data available on the socket
    read_sockets, _, _ = select.select([self._server_socket], [], [], 0)
    if not read_sockets:
      return []

    # Since we only ever pass the server socket to the list of read sockets, we can just read from that
    # Read all available data into a buffer
    data = b''
    while True:
      try:
        chunk = self._server_socket.recv(_CHUNK_SIZE)
        data += chunk
        if len(chunk) < _CHUNK_SIZE:
          break
      except Exception as e:
        self._logerr('Error while reading {} bytes from socket'.format(_CHUNK_SIZE))
        if not self._socket_is_open():
          self._logerr('Socket appears to be closed. Reconnecting')
          self.request_reconnect()
          return []
        break
    self._logdebug('Read {} bytes'.format(len(data)))

    # If 0 bytes were read from the socket even though we were told data is available multiple times,
    # it can be safely assumed that we can reconnect as the server has closed the connection
    if len(data) == 0:
      self._read_zero_bytes_count += 1
      if self._read_zero_bytes_count >= self._read_zero_bytes_max:
        self._logwarn('Reconnecting because we received 0 bytes from the socket even though it said there was data available {} times'.format(self._read_zero_bytes_count))
        self.request_reconnect()
        self._read_zero_bytes_count = 0
        return []
    else:
      # Looks like we received valid data, so note when the data was received
      self._recv_rtcm_last_packet_timestamp = time.time()
      self._first_rtcm_received = True

    # Send the data to the RTCM parser to parse it
    return self.rtcm_parser.parse(data) if data else []

  def shutdown(self):
    # Set some state, and then disconnect
    self._shutdown = True
    self.disconnect()

  def _form_request(self):
    if self._ntrip_version != None and self._ntrip_version != '':
      request_str = 'GET /{} HTTP/1.0\r\nNtrip-Version: {}\r\nUser-Agent: {}\r\n'.format(
        self._mountpoint, self._ntrip_version, self._user_agent)
    else:
      request_str = 'GET /{} HTTP/1.0\r\nUser-Agent: {}\r\n'.format(
        self._mountpoint, self._user_agent)
    if self._basic_credentials is not None:
      request_str += 'Authorization: Basic {}\r\n'.format(
        self._basic_credentials)
    request_str += '\r\n'
    return request_str.encode('utf-8')

  def _socket_is_open(self):
    try:
      # this will try to read bytes without blocking and also without removing them from buffer (peek only)
      data = self._server_socket.recv(_CHUNK_SIZE, socket.MSG_DONTWAIT | socket.MSG_PEEK)
      if len(data) == 0:
        return False
    except BlockingIOError:
      return True  # socket is open and reading from it would block
    except ConnectionResetError:
      self._logwarn('Connection reset by peer')
      return False  # socket was closed for some other reason
    except socket.timeout:
      return True  # timeout likely means that the socket is still open
    except Exception as e:
      self._logwarn('Socket appears to be closed')
      self._logwarn('Exception: {}'.format(e))
      return False
    return True
