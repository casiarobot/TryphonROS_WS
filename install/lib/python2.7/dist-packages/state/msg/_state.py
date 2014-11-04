"""autogenerated by genpy from state/state.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class state(genpy.Message):
  _md5sum = "ffedcd09c7959513979d11f3021f4fde"
  _type = "state/state"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """float64[3] pos
float64[3] vel
float64[4] quat
float64[3] angvel

"""
  __slots__ = ['pos','vel','quat','angvel']
  _slot_types = ['float64[3]','float64[3]','float64[4]','float64[3]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       pos,vel,quat,angvel

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(state, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.pos is None:
        self.pos = [0.,0.,0.]
      if self.vel is None:
        self.vel = [0.,0.,0.]
      if self.quat is None:
        self.quat = [0.,0.,0.,0.]
      if self.angvel is None:
        self.angvel = [0.,0.,0.]
    else:
      self.pos = [0.,0.,0.]
      self.vel = [0.,0.,0.]
      self.quat = [0.,0.,0.,0.]
      self.angvel = [0.,0.,0.]

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
      buff.write(_struct_3d.pack(*self.pos))
      buff.write(_struct_3d.pack(*self.vel))
      buff.write(_struct_4d.pack(*self.quat))
      buff.write(_struct_3d.pack(*self.angvel))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      start = end
      end += 24
      self.pos = _struct_3d.unpack(str[start:end])
      start = end
      end += 24
      self.vel = _struct_3d.unpack(str[start:end])
      start = end
      end += 32
      self.quat = _struct_4d.unpack(str[start:end])
      start = end
      end += 24
      self.angvel = _struct_3d.unpack(str[start:end])
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
      buff.write(self.pos.tostring())
      buff.write(self.vel.tostring())
      buff.write(self.quat.tostring())
      buff.write(self.angvel.tostring())
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      start = end
      end += 24
      self.pos = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=3)
      start = end
      end += 24
      self.vel = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=3)
      start = end
      end += 32
      self.quat = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=4)
      start = end
      end += 24
      self.angvel = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=3)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_4d = struct.Struct("<4d")
_struct_3d = struct.Struct("<3d")