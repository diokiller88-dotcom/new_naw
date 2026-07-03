# generated from rosidl_generator_py/resource/_idl.py.em
# with input from custom_msgs:msg/ChassisInfo.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ChassisInfo(type):
    """Metaclass of message 'ChassisInfo'."""

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
                'custom_msgs.msg.ChassisInfo')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__chassis_info
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__chassis_info
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__chassis_info
            cls._TYPE_SUPPORT = module.type_support_msg__msg__chassis_info
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__chassis_info

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


class ChassisInfo(metaclass=Metaclass_ChassisInfo):
    """Message class 'ChassisInfo'."""

    __slots__ = [
        '_header',
        '_is_valid',
        '_trigger_relocation',
        '_trigger_target',
        '_speed',
        '_target_x',
        '_target_y',
        '_gimbal_yaw',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'is_valid': 'boolean',
        'trigger_relocation': 'boolean',
        'trigger_target': 'boolean',
        'speed': 'double',
        'target_x': 'double',
        'target_y': 'double',
        'gimbal_yaw': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
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
        self.trigger_relocation = kwargs.get('trigger_relocation', bool())
        self.trigger_target = kwargs.get('trigger_target', bool())
        self.speed = kwargs.get('speed', float())
        self.target_x = kwargs.get('target_x', float())
        self.target_y = kwargs.get('target_y', float())
        self.gimbal_yaw = kwargs.get('gimbal_yaw', float())

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
        if self.trigger_relocation != other.trigger_relocation:
            return False
        if self.trigger_target != other.trigger_target:
            return False
        if self.speed != other.speed:
            return False
        if self.target_x != other.target_x:
            return False
        if self.target_y != other.target_y:
            return False
        if self.gimbal_yaw != other.gimbal_yaw:
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
    def trigger_relocation(self):
        """Message field 'trigger_relocation'."""
        return self._trigger_relocation

    @trigger_relocation.setter
    def trigger_relocation(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'trigger_relocation' field must be of type 'bool'"
        self._trigger_relocation = value

    @builtins.property
    def trigger_target(self):
        """Message field 'trigger_target'."""
        return self._trigger_target

    @trigger_target.setter
    def trigger_target(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'trigger_target' field must be of type 'bool'"
        self._trigger_target = value

    @builtins.property
    def speed(self):
        """Message field 'speed'."""
        return self._speed

    @speed.setter
    def speed(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'speed' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'speed' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._speed = value

    @builtins.property
    def target_x(self):
        """Message field 'target_x'."""
        return self._target_x

    @target_x.setter
    def target_x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'target_x' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'target_x' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._target_x = value

    @builtins.property
    def target_y(self):
        """Message field 'target_y'."""
        return self._target_y

    @target_y.setter
    def target_y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'target_y' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'target_y' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._target_y = value

    @builtins.property
    def gimbal_yaw(self):
        """Message field 'gimbal_yaw'."""
        return self._gimbal_yaw

    @gimbal_yaw.setter
    def gimbal_yaw(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'gimbal_yaw' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'gimbal_yaw' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._gimbal_yaw = value
