"""autogenerated by genmsg_py from sonar_params.msg. Do not edit."""
import roslib.message
import struct

import roslib.rostime

class sonar_params(roslib.message.Message):
  _md5sum = "d1540bdf311a1a9a5f49f64608ceca0a"
  _type = "usbi2c/sonar_params"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """int32 index
int32 range_mm
time stamp

"""
  __slots__ = ['index','range_mm','stamp']
  _slot_types = ['int32','int32','time']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.
    
    The available fields are:
       index,range_mm,stamp
    
    @param args: complete set of field values, in .msg order
    @param kwds: use keyword arguments corresponding to message field names
    to set specific fields. 
    """
    if args or kwds:
      super(sonar_params, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.index is None:
        self.index = 0
      if self.range_mm is None:
        self.range_mm = 0
      if self.stamp is None:
        self.stamp = roslib.rostime.Time()
    else:
      self.index = 0
      self.range_mm = 0
      self.stamp = roslib.rostime.Time()

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    @param buff: buffer
    @type  buff: StringIO
    """
    try:
      _x = self
      buff.write(_struct_2i2I.pack(_x.index, _x.range_mm, _x.stamp.secs, _x.stamp.nsecs))
    except struct.error, se: self._check_types(se)
    except TypeError, te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    @param str: byte array of serialized message
    @type  str: str
    """
    try:
      if self.stamp is None:
        self.stamp = roslib.rostime.Time()
      end = 0
      _x = self
      start = end
      end += 16
      (_x.index, _x.range_mm, _x.stamp.secs, _x.stamp.nsecs,) = _struct_2i2I.unpack(str[start:end])
      self.stamp.canon()
      return self
    except struct.error, e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    @param buff: buffer
    @type  buff: StringIO
    @param numpy: numpy python module
    @type  numpy module
    """
    try:
      _x = self
      buff.write(_struct_2i2I.pack(_x.index, _x.range_mm, _x.stamp.secs, _x.stamp.nsecs))
    except struct.error, se: self._check_types(se)
    except TypeError, te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    @param str: byte array of serialized message
    @type  str: str
    @param numpy: numpy python module
    @type  numpy: module
    """
    try:
      if self.stamp is None:
        self.stamp = roslib.rostime.Time()
      end = 0
      _x = self
      start = end
      end += 16
      (_x.index, _x.range_mm, _x.stamp.secs, _x.stamp.nsecs,) = _struct_2i2I.unpack(str[start:end])
      self.stamp.canon()
      return self
    except struct.error, e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill

_struct_I = roslib.message.struct_I
_struct_2i2I = struct.Struct("<2i2I")