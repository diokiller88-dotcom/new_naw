# generated from rosidl_generator_py/resource/_idl.py.em
# with input from custom_msgs:msg/Result.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Result(type):
    """Metaclass of message 'Result'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('custom_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'custom_msgs.msg.Result')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__result
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__result
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__result
            cls._TYPE_SUPPORT = module.type_support_msg__msg__result
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__result

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Result(metaclass=Metaclass_Result):
    """Message class 'Result'."""

    __slots__ = [
        '_header',
        '_is_valid',
        '_res_pose_x',
        '_res_pose_y',
        '_res_vel_x',
        '_res_vel_y',
        '_yaw_diff',
        '_vyaw',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'is_valid': 'boolean',
        'res_pose_x': 'double',
        'res_pose_y': 'double',
        'res_vel_x': 'double',
        'res_vel_y': 'double',
        'yaw_diff': 'double',
        'vyaw': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.is_valid = kwargs.get('is_valid', bool())
        self.res_pose_x = kwargs.get('res_pose_x', float())
        self.res_pose_y = kwargs.get('res_pose_y', float())
        self.res_vel_x = kwargs.get('res_vel_x', float())
        self.res_vel_y = kwargs.get('res_vel_y', float())
        self.yaw_diff = kwargs.get('yaw_diff', float())
        self.vyaw = kwargs.get('vyaw', float())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.header != other.header:
            return False
        if self.is_valid != other.is_valid:
            return False
        if self.res_pose_x != other.res_pose_x:
            return False
        if self.res_pose_y != other.res_pose_y:
            return False
        if self.res_vel_x != other.res_vel_x:
            return False
        if self.res_vel_y != other.res_vel_y:
            return False
        if self.yaw_diff != other.yaw_diff:
            return False
        if self.vyaw != other.vyaw:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if __debug__:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @builtins.property
    def is_valid(self):
        """Message field 'is_valid'."""
        return self._is_valid

    @is_valid.setter
    def is_valid(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'is_valid' field must be of type 'bool'"
        self._is_valid = value

    @builtins.property
    def res_pose_x(self):
        """Message field 'res_pose_x'."""
        return self._res_pose_x

    @res_pose_x.setter
    def res_pose_x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'res_pose_x' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'res_pose_x' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._res_pose_x = value

    @builtins.property
    def res_pose_y(self):
        """Message field 'res_pose_y'."""
        return self._res_pose_y

    @res_pose_y.setter
    def res_pose_y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'res_pose_y' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'res_pose_y' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._res_pose_y = value

    @builtins.property
    def res_vel_x(self):
        """Message field 'res_vel_x'."""
        return self._res_vel_x

    @res_vel_x.setter
    def res_vel_x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'res_vel_x' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'res_vel_x' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._res_vel_x = value

    @builtins.property
    def res_vel_y(self):
        """Message field 'res_vel_y'."""
        return self._res_vel_y

    @res_vel_y.setter
    def res_vel_y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'res_vel_y' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'res_vel_y' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._res_vel_y = value

    @builtins.property
    def yaw_diff(self):
        """Message field 'yaw_diff'."""
        return self._yaw_diff

    @yaw_diff.setter
    def yaw_diff(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'yaw_diff' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'yaw_diff' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._yaw_diff = value

    @builtins.property
    def vyaw(self):
        """Message field 'vyaw'."""
        return self._vyaw

    @vyaw.setter
    def vyaw(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'vyaw' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'vyaw' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._vyaw = value
