# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from rbt_baxter_msgs/EndEffectorCommand.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class EndEffectorCommand(genpy.Message):
  _md5sum = "c003234e90416f2ca02ac7837c42cbb7"
  _type = "rbt_baxter_msgs/EndEffectorCommand"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """## Command to be sent to an end effector
uint32 id       # target end effector id
string command  # operation to perform
# Well known commands:
string   CMD_NO_OP           = no_op
string   CMD_SET             = set
string   CMD_CONFIGURE       = configure
string   CMD_REBOOT          = reboot
string   CMD_RESET           = reset
string   CMD_CALIBRATE       = calibrate
string   CMD_CLEAR_CALIBRATION = clear_calibration
string   CMD_PREPARE_TO_GRIP = prepare_to_grip
string   CMD_GRIP            = grip
string   CMD_RELEASE         = release
string   CMD_GO              = go
string   CMD_STOP            = stop
#
string args     # JSON arguments to the command
#
string sender   # optional identifier, returned in state when the command is handled
uint32 sequence # optional sequence number, return in state when the command is handled

"""
  # Pseudo-constants
  CMD_NO_OP = 'no_op'
  CMD_SET = 'set'
  CMD_CONFIGURE = 'configure'
  CMD_REBOOT = 'reboot'
  CMD_RESET = 'reset'
  CMD_CALIBRATE = 'calibrate'
  CMD_CLEAR_CALIBRATION = 'clear_calibration'
  CMD_PREPARE_TO_GRIP = 'prepare_to_grip'
  CMD_GRIP = 'grip'
  CMD_RELEASE = 'release'
  CMD_GO = 'go'
  CMD_STOP = 'stop'

  __slots__ = ['id','command','args','sender','sequence']
  _slot_types = ['uint32','string','string','string','uint32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       id,command,args,sender,sequence

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(EndEffectorCommand, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.id is None:
        self.id = 0
      if self.command is None:
        self.command = ''
      if self.args is None:
        self.args = ''
      if self.sender is None:
        self.sender = ''
      if self.sequence is None:
        self.sequence = 0
    else:
      self.id = 0
      self.command = ''
      self.args = ''
      self.sender = ''
      self.sequence = 0

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
      buff.write(_struct_I.pack(self.id))
      _x = self.command
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      if python3:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.args
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      if python3:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.sender
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      if python3:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
      buff.write(_struct_I.pack(self.sequence))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      start = end
      end += 4
      (self.id,) = _struct_I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.command = str[start:end].decode('utf-8')
      else:
        self.command = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.args = str[start:end].decode('utf-8')
      else:
        self.args = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.sender = str[start:end].decode('utf-8')
      else:
        self.sender = str[start:end]
      start = end
      end += 4
      (self.sequence,) = _struct_I.unpack(str[start:end])
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
      buff.write(_struct_I.pack(self.id))
      _x = self.command
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      if python3:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.args
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      if python3:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.sender
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      if python3:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
      buff.write(_struct_I.pack(self.sequence))
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
      start = end
      end += 4
      (self.id,) = _struct_I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.command = str[start:end].decode('utf-8')
      else:
        self.command = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.args = str[start:end].decode('utf-8')
      else:
        self.args = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.sender = str[start:end].decode('utf-8')
      else:
        self.sender = str[start:end]
      start = end
      end += 4
      (self.sequence,) = _struct_I.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
