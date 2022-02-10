import logging

_NMEA_MAX_LENGTH = 82
_NMEA_MIN_LENGTH = 3
_NMEA_CHECKSUM_SEPERATOR = "*"

class NMEAParser:

  def __init__(self, logerr=logging.error, logwarn=logging.warning, loginfo=logging.info, logdebug=logging.debug):
    # Bit of a strange pattern here, but save the log functions so we can be agnostic of ROS
    self._logerr = logerr
    self._logwarn = logwarn
    self._loginfo = loginfo
    self._logdebug = logdebug

  def is_valid_sentence(self, sentence):
    # Simple sanity checks
    if len(sentence) > _NMEA_MAX_LENGTH:
      self._logwarn('Received invalid NMEA sentence. Max length is {}, but sentence was {} bytes'.format(_NMEA_MAX_LENGTH, len(sentence)))
      self._logwarn('Sentence: {}'.format(sentence))
      return False
    if len(sentence) < _NMEA_MIN_LENGTH:
      self._logwarn('Received invalid NMEA sentence. We need at least {} bytes to parse but got {} bytes'.format(_NMEA_MIN_LENGTH, len(sentence)))
      self._logwarn('Sentence: {}'.format(sentence))
      return False
    if sentence[0] != '$' and sentence[0] != '!':
      self._logwarn('Received invalid NMEA sentence. Sentence should begin with "$" or "!", but instead begins with {}'.format(sentence[0]))
      self._logwarn('Sentence: {}'.format(sentence))
      return False
    if sentence[-2:] != '\r\n':
      self._logwarn('Received invalid NMEA sentence. Sentence should end with \\r\\n, but instead ends with {}'.format(sentence[-2:]))
      self._logwarn('Sentence: {}'.format(sentence))
      return False
    if _NMEA_CHECKSUM_SEPERATOR not in sentence:
      self._logwarn('Received invalid NMEA sentence. Sentence should have a "{}" character to seperate the checksum, but we could not find it.'.format(_NMEA_CHECKSUM_SEPERATOR))
      self._logwarn('Sentence: {}'.format(sentence))
      return False

    # Checksum check
    data, expected_checksum_str = sentence.rsplit(_NMEA_CHECKSUM_SEPERATOR, 1)
    expected_checksum = int(expected_checksum_str, 16)
    calculated_checksum = 0
    for char in data[1:]:
      calculated_checksum ^= ord(char)
    if expected_checksum != calculated_checksum:
      self._logwarn('Received invalid NMEA sentence. Checksum mismatch');
      self._logwarn('Expected Checksum:   0x{:X}'.format(expected_checksum))
      self._logwarn('Calculated Checksum: 0x{:X}'.format(calculated_checksum))
      return False

    # Passed all checks
    return True
