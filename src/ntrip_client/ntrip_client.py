#!/usr/bin/env python

import base64
import socket
import select
import logging

from .nmea_parser import NMEAParser
from .rtcm_parser import RTCMParser

_CHUNK_SIZE = 1024
_SOURCETABLE_RESPONSES = [
  'SOURCETABLE 200 OK'
]
_SUCCESS_RESPONSES = [
  'ICY 200 OK',
  'HTTP/1.0 200 OK'
]
_UNAUTHORIZED_RESPONSES = [
  '401'
]


class NTRIPClient:

  def __init__(self, host, port, mountpoint, ntrip_version, username, password, logerr=logging.error, logwarn=logging.warning, loginfo=logging.info, logdebug=logging.debug):
    # Bit of a strange pattern here, but save the log functions so we can be agnostic of ROS
    self._logerr = logerr
    self._logwarn = logwarn
    self._loginfo = loginfo
    self._logdebug = logdebug

    # Save the server info
    self._host = host
    self._port = port
    self._mountpoint = mountpoint
    self._ntrip_version = ntrip_version
    if username is not None and password is not None:
      self._basic_credentials = base64.b64encode('{}:{}'.format(
        username, password).encode('utf-8')).decode('utf-8')
    else:
      self._basic_credentials = None

    # Create a socket object that we will use to connect to the server
    self._server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # Setup some parsers to parse incoming messages
    self._rtcm_parser = RTCMParser(
      logerr=logerr,
      logwarn=logwarn,
      loginfo=loginfo,
      logdebug=logdebug
    )
    self._nmea_parser = NMEAParser(
      logerr=logerr,
      logwarn=logwarn,
      loginfo=loginfo,
      logdebug=logdebug
    )

    # Setup some state
    self._connected = False

  def connect(self):
    # Connect the socket to the server
    try:
      self._server_socket.connect((self._host, self._port))
    except Exception as e:
      self._logerr(
        'Unable to connect socket to server at http://{}:{}'.format(self._host, self._port))
      self._logerr('Exception: {}'.format(str(e)))
      return False

    # Send the HTTP Request
    try:
      self._server_socket.send(self._form_request())
    except Exception as e:
      self._logerr(
        'Unable to send request to server at http://{}:{}'.format(self._host, self._port))
      self._logerr('Exception: {}'.format(str(e)))
      return False

    # Get the response from the server
    response = ''
    try:
      response = self._server_socket.recv(1024).decode('utf-8')
    except Exception as e:
      self._logerr(
        'Unable to read response from server at http://{}:{}'.format(self._host, self._port))
      self._logerr('Exception: {}'.format(str(e)))
      return False

    # Properly handle the response
    if any(success in response for success in _SUCCESS_RESPONSES):
      self._server_socket.setblocking(False)
      self._connected = True

    # Some debugging hints about the kind of error we received
    known_error = False
    if any(sourcetable in response for sourcetable in _SOURCETABLE_RESPONSES):
      self._logwarn('Received sourcetable response from the server. This probably means the mountpoint specified is not valid')
      known_error = True
    elif any(unauthorized in response for unauthorized in _UNAUTHORIZED_RESPONSES):
      self._logwarn('Received unauthorized response from the server. Check your username, password, and mountpoint to make sure they are correct.')
      known_error = True
    elif not self._connected and (self._ntrip_version == None or self._ntrip_version == ''):
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
    self._server_socket.close()
    self._connected = False

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
    if not self._nmea_parser.is_valid_sentence(sentence):
      self._logwarn("Invalid NMEA sentence, not sending to server")
      return

    # Encode the data and send it to the socket
    try:
      self._server_socket.send(sentence.encode('utf-8'))
    except Exception as e:
      self._logwarn('Unable to send NMEA sentence to server.')
      self._logwarn('Exception: {}'.format(str(e)))

  def recv_rtcm(self):
    if not self._connected:
      self._logwarn(
        'RTCM requested before client was connected, returning empty list')
      return []

    # Check if there is any data available on the socket
    read_sockets, _, _ = select.select([self._server_socket], [], [], 0)
    if not read_sockets:
      return []

    # Since we only ever pass the server socket to the list of read sockets, we can just read from that
    # Read all available data into a buffer
    data = b''
    while True:
      chunk = self._server_socket.recv(_CHUNK_SIZE)
      data += chunk
      if len(chunk) < _CHUNK_SIZE:
        break
    self._logdebug('Read {} bytes'.format(len(data)))

    # Send the data to the RTCM parser to parse it
    return self._rtcm_parser.parse(data) if data else []

  def _form_request(self):
    if self._ntrip_version != None and self._ntrip_version != '':
      request_str = 'GET /{} HTTP/1.0\r\nNtrip-Version: {}\r\nUser-Agent: NTRIP ntrip_client_ros\r\n'.format(
        self._mountpoint, self._ntrip_version)
    else:
      request_str = 'GET /{} HTTP/1.0\r\nUser-Agent: NTRIP ntrip_client_ros\r\n'.format(
        self._mountpoint)
    if self._basic_credentials is not None:
      request_str += 'Authorization: Basic {}\r\n'.format(
        self._basic_credentials)
    request_str += '\r\n'
    return request_str.encode('utf-8')
