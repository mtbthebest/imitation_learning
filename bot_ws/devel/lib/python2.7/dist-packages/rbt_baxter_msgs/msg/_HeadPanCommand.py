# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from rbt_baxter_msgs/HeadPanCommand.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class HeadPanCommand(genpy.Message):
  _md5sum = "23b8a3f4b7ee9de7099d029e57660a8c"
  _type = "rbt_baxter_msgs/HeadPanCommand"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """float32 target              # radians for target, 0 str
float32 speed_ratio         # Percentage of max speed [0-1]
#
  float32 MAX_SPEED_RATIO = 1.0
  float32 MIN_SPEED_RATIO = 0.0
#
uint8   enable_pan_request  # override automatic pan enable/disable
# enable_pan_request is tristate: 0 = disable pan; 1 = enable pan; 2 = don't change pan
  uint8   REQUEST_PAN_DISABLE = 0
  uint8   REQUEST_PAN_ENABLE = 1
  uint8   REQUEST_PAN_VOID = 2
#
"""
  # Pseudo-constants
  MAX_SPEED_RATIO = 1.0
  MIN_SPEED_RATIO = 0.0
  REQUEST_PAN_DISABLE = 0
  REQUEST_PAN_ENABLE = 1
  REQUEST_PAN_VOID = 2

  __slots__ = ['target','speed_ratio','enable_pan_request']
  _slot_types = ['float32','float32','uint8']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       target,speed_ratio,enable_pan_request

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(HeadPanCommand, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.target is None:
        self.target = 0.
      if self.speed_ratio is None:
        self.speed_ratio = 0.
      if self.enable_pan_request is None:
        self.enable_pan_request = 0
    else:
      self.target = 0.
      self.speed_ratio = 0.
      self.enable_pan_request = 0

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
      buff.write(_struct_2fB.pack(_x.target, _x.speed_ratio, _x.enable_pan_request))
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
      end += 9
      (_x.target, _x.speed_ratio, _x.enable_pan_request,) = _struct_2fB.unpack(str[start:end])
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
      buff.write(_struct_2fB.pack(_x.target, _x.speed_ratio, _x.enable_pan_request))
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
      end += 9
      (_x.target, _x.speed_ratio, _x.enable_pan_request,) = _struct_2fB.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_2fB = struct.Struct("<2fB")
