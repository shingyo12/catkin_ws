# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from ublox_msgs/CfgCFG.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class CfgCFG(genpy.Message):
  _md5sum = "82e4847c642bca8fb5b8af4595e063a1"
  _type = "ublox_msgs/CfgCFG"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """# CFG-CFG (0x06 0x09)
# Clear, Save and Load configurations
#

uint8 CLASS_ID = 6
uint8 MESSAGE_ID = 9

uint32 clearMask          # Mask with configuration sub-sections to Clear
                          # (=Load Default Configurations to Permanent
                          # Configurations in non-volatile memory)
uint32 saveMask           # Mask with configuration sub-section to Save
                          # (=Save Current Configuration to Non-volatile
                          # Memory)
uint32 loadMask           # Mask with configuration sub-sections to Load
                          # (=Load Permanent Configurations from
                          # Non-volatile Memory to Current Configurations)

uint32 MASK_IO_PORT = 1       # Communications port settings. Modifying this 
                              # sub-section results in an IO system reset. 
                              # Because of this undefined data may be output 
                              # for a short period of time after receiving the
                              # message.
uint32 MASK_MSG_CONF = 2      # Message Configuration
uint32 MASK_INF_MSG = 4       # INF Message Configuration
uint32 MASK_NAV_CONF = 8      # Navigation Configuration
uint32 MASK_RXM_CONF = 16     # Receiver Manager Configuration
uint32 MASK_SEN_CONF = 256    # Sensor Interface Configuration, protocol >= 19
uint32 MASK_RINV_CONF = 512   # Remote Inventory Configuration
uint32 MASK_ANT_CONF = 1024   # Antenna Configuration
uint32 MASK_LOG_CONF = 2048   # Logging Configuration
uint32 MASK_FTS_CONF = 4096   # FTS Configuration. Only applicable to the 
                              # FTS product variant.

uint8 deviceMask          # Mask which selects the devices for this command
uint8 DEV_BBR = 1             # device battery backed RAM
uint8 DEV_FLASH = 2           # device Flash
uint8 DEV_EEPROM = 4          # device EEPROM
uint8 DEV_SPI_FLASH = 16      # device SPI Flash
"""
  # Pseudo-constants
  CLASS_ID = 6
  MESSAGE_ID = 9
  MASK_IO_PORT = 1
  MASK_MSG_CONF = 2
  MASK_INF_MSG = 4
  MASK_NAV_CONF = 8
  MASK_RXM_CONF = 16
  MASK_SEN_CONF = 256
  MASK_RINV_CONF = 512
  MASK_ANT_CONF = 1024
  MASK_LOG_CONF = 2048
  MASK_FTS_CONF = 4096
  DEV_BBR = 1
  DEV_FLASH = 2
  DEV_EEPROM = 4
  DEV_SPI_FLASH = 16

  __slots__ = ['clearMask','saveMask','loadMask','deviceMask']
  _slot_types = ['uint32','uint32','uint32','uint8']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       clearMask,saveMask,loadMask,deviceMask

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(CfgCFG, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.clearMask is None:
        self.clearMask = 0
      if self.saveMask is None:
        self.saveMask = 0
      if self.loadMask is None:
        self.loadMask = 0
      if self.deviceMask is None:
        self.deviceMask = 0
    else:
      self.clearMask = 0
      self.saveMask = 0
      self.loadMask = 0
      self.deviceMask = 0

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_3IB().pack(_x.clearMask, _x.saveMask, _x.loadMask, _x.deviceMask))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      _x = self
      start = end
      end += 13
      (_x.clearMask, _x.saveMask, _x.loadMask, _x.deviceMask,) = _get_struct_3IB().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_3IB().pack(_x.clearMask, _x.saveMask, _x.loadMask, _x.deviceMask))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      _x = self
      start = end
      end += 13
      (_x.clearMask, _x.saveMask, _x.loadMask, _x.deviceMask,) = _get_struct_3IB().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_3IB = None
def _get_struct_3IB():
    global _struct_3IB
    if _struct_3IB is None:
        _struct_3IB = struct.Struct("<3IB")
    return _struct_3IB
