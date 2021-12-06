#!/usr/bin/env python

import base64
import socket
import select
import logging

from .nmea_parser import NMEAParser
from .rtcm_parser import RTCMParser

_CHUNK_SIZE = 1024
_SOURCETABLE_RESPONSE = 'SOURCETABLE 200 OK'
_SUCCESS_RESPONSE = 'ICY 200 OK'


class NTRIPClient:

  def __init__(self, host, port, mountpoint, username, password, logerr=logging.error, logwarn=logging.warning, loginfo=logging.info, logdebug=logging.debug):
    # Bit of a strange pattern here, but save the log functions so we can be agnostic of ROS
    self._logerr = logerr
    self._logwarn = logwarn
    self._loginfo = loginfo
    self._logdebug = logdebug

    # Save the server info
    self._host = host
    self._port = port
    self._mountpoint = mountpoint
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
    if _SUCCESS_RESPONSE in response:
      self._loginfo(
        'Connected to http://{}:{}/{}'.format(self._host, self._port, self._mountpoint))
      self._server_socket.setblocking(False)
      self._connected = True
      return True
    elif _SOURCETABLE_RESPONSE in response:
      self._logerr('Received sourcetable from http://{}:{}/{}, this probably means that the mountpoint used is invalid'.format(
        self._host, self._port, self._mountpoint))
      self._logerr('Sourcetable Response: \n{}'.format(response))
      return False
    else:
      self._logerr('Invalid response received from http://{}:{}/{}'.format(
        self._host, self._port, self._mountpoint))
      self._logerr('Response: {}'.format(response))
      return False

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
    request_str = 'GET /{} HTTP/1.0\r\nNtrip-Version\r\nUser-Agent: NTRIP ntrip_client_ros\r\n'.format(
      self._mountpoint)
    if self._basic_credentials is not None:
      request_str += 'Authorization: Basic {}\r\n'.format(
        self._basic_credentials)
    request_str += '\r\n'
    return request_str.encode('utf-8')
