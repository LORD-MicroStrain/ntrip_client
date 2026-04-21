import math
import logging

NMEA_DEFAULT_MAX_LENGTH = 82
NMEA_DEFAULT_MIN_LENGTH = 3
_NMEA_CHECKSUM_SEPERATOR = "*"

class NMEAParser:

  def __init__(self, logerr=logging.error, logwarn=logging.warning, loginfo=logging.info, logdebug=logging.debug):
    # Bit of a strange pattern here, but save the log functions so we can be agnostic of ROS
    self._logerr = logerr
    self._logwarn = logwarn
    self._loginfo = loginfo
    self._logdebug = logdebug

    # Save some other config
    self.nmea_max_length = NMEA_DEFAULT_MAX_LENGTH
    self.nmea_min_length = NMEA_DEFAULT_MIN_LENGTH

    self._caching_data = False
    self._buffer = b''

  @staticmethod
  def checksum(sentence_no_checksum):
    calculated_checksum = 0
    for char in sentence_no_checksum[1:]:
      calculated_checksum ^= ord(char)
    return calculated_checksum
  
  @staticmethod
  def lat_dd_to_dmm(decimal_degrees):
    decimal_degrees_sub, decimal_degrees_whole = math.modf(abs(decimal_degrees))
    decimal_minutes_float = decimal_degrees_sub * 60
    decimal_minutes_sub, decimal_minutes_whole = math.modf(decimal_minutes_float)
    degrees = int(decimal_degrees_whole)
    decimal_minutes = int(decimal_minutes_whole)
    decimal_subminutes = int(decimal_minutes_sub * 1e5)
    return f"{degrees:02}{decimal_minutes:02}.{decimal_subminutes:02}"

  @staticmethod
  def lon_dd_to_dmm(decimal_degrees):
    decimal_degrees_sub, decimal_degrees_whole = math.modf(abs(decimal_degrees))
    decimal_minutes_float = decimal_degrees_sub * 60
    decimal_minutes_sub, decimal_minutes_whole = math.modf(decimal_minutes_float)
    degrees = int(decimal_degrees_whole)
    decimal_minutes = int(decimal_minutes_whole)
    decimal_subminutes = int(decimal_minutes_sub * 1e5)
    return f"{degrees:03}{decimal_minutes:02}.{decimal_subminutes:02}"

  def is_valid_sentence(self, sentence):
    # Simple sanity checks
    if len(sentence) > self.nmea_max_length:
      self._logwarn('Received invalid NMEA sentence. Max length is {}, but sentence was {} bytes'.format(self.nmea_max_length, len(sentence)))
      self._logwarn('Sentence: {}'.format(sentence))
      return False
    if len(sentence) < self.nmea_min_length:
      self._logwarn('Received invalid NMEA sentence. We need at least {} bytes to parse but got {} bytes'.format(self.nmea_min_length, len(sentence)))
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
    calculated_checksum = self.checksum(data)
    if expected_checksum != calculated_checksum:
      self._logwarn('Received invalid NMEA sentence. Checksum mismatch');
      self._logwarn('Expected Checksum:   0x{:X}'.format(expected_checksum))
      self._logwarn('Calculated Checksum: 0x{:X}'.format(calculated_checksum))
      return False

    # Passed all checks
    return True
    
  def parse(self, buffer):
    # Add any data we have cached
    if self._caching_data:
      combined_buffer = self._buffer + buffer
    else:
      combined_buffer = buffer
      
      # Loop over the passed buffer, and parse all available NMEA sentences
    index = 0
    nmea_sentences = []
    while index < len(combined_buffer):
      # Find the start of the NMEA sentence
      if combined_buffer[index] == ord('$'):
        # Check if we have the end of the sentence in the buffer, and if so, try to parse it
        end_index = combined_buffer.find(b'\r\n', index)
        if end_index != -1:
          sentence = combined_buffer[index:end_index + 2].decode('utf-8')
          if self.is_valid_sentence(sentence):
            nmea_sentences.append(sentence)
          else:
            self._logwarn('Found potential NMEA sentence, but it was invalid. Sentence: {}'.format(sentence))
          index = end_index + 2
        else:
          self._logdebug('Found beginning of NMEA sentence at {}, but there is not enough data in the buffer to find the end of the sentence'.format(index))
          self._caching_data = True
          self._buffer = buffer[index:]
          break
      index += 1

    # If we didn't find a full sentence, cache this one for next time
    if self._caching_data:
      self._buffer += buffer  
    
      if len(self._buffer) > self.nmea_max_length:
        self._logwarn('Too much data buffered, trimming to {} bytes.'.format(self.nmea_max_length))
        self._buffer = self._buffer[:self.nmea_max_length]
    
    # Return the NMEA sentences we found
    return nmea_sentences 
    