"""autogenerated by genpy from sensors/imuros.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class imuros(genpy.Message):
  _md5sum = "b01df9aeabf2ec2a45aad9d35856770e"
  _type = "sensors/imuros"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """float64[3] accel
float64[3] gyro
float64[3] magn

"""
  __slots__ = ['accel','gyro','magn']
  _slot_types = ['float64[3]','float64[3]','float64[3]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       accel,gyro,magn

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(imuros, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.accel is None:
        self.accel = [0.,0.,0.]
      if self.gyro is None:
        self.gyro = [0.,0.,0.]
      if self.magn is None:
        self.magn = [0.,0.,0.]
    else:
      self.accel = [0.,0.,0.]
      self.gyro = [0.,0.,0.]
      self.magn = [0.,0.,0.]

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
      buff.write(_struct_3d.pack(*self.accel))
      buff.write(_struct_3d.pack(*self.gyro))
      buff.write(_struct_3d.pack(*self.magn))
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
      self.accel = _struct_3d.unpack(str[start:end])
      start = end
      end += 24
      self.gyro = _struct_3d.unpack(str[start:end])
      start = end
      end += 24
      self.magn = _struct_3d.unpack(str[start:end])
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
      buff.write(self.accel.tostring())
      buff.write(self.gyro.tostring())
      buff.write(self.magn.tostring())
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
      self.accel = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=3)
      start = end
      end += 24
      self.gyro = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=3)
      start = end
      end += 24
      self.magn = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=3)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_3d = struct.Struct("<3d")
