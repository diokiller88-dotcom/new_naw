# generated from rosidl_generator_py/resource/_idl.py.em
# with input from custom_msgs:msg/MapData.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'occupancy_array'
# Member 'esdf_array'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_MapData(type):
    """Metaclass of message 'MapData'."""

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
                'custom_msgs.msg.MapData')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__map_data
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__map_data
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__map_data
            cls._TYPE_SUPPORT = module.type_support_msg__msg__map_data
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__map_data

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


class MapData(metaclass=Metaclass_MapData):
    """Message class 'MapData'."""

    __slots__ = [
        '_header',
        '_is_valid',
        '_width',
        '_height',
        '_res_x',
        '_res_y',
        '_origin_x',
        '_origin_y',
        '_occupancy_array',
        '_esdf_array',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'is_valid': 'boolean',
        'width': 'int32',
        'height': 'int32',
        'res_x': 'double',
        'res_y': 'double',
        'origin_x': 'double',
        'origin_y': 'double',
        'occupancy_array': 'sequence<int32>',
        'esdf_array': 'sequence<double>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('int32')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('double')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.is_valid = kwargs.get('is_valid', bool())
        self.width = kwargs.get('width', int())
        self.height = kwargs.get('height', int())
        self.res_x = kwargs.get('res_x', float())
        self.res_y = kwargs.get('res_y', float())
        self.origin_x = kwargs.get('origin_x', float())
        self.origin_y = kwargs.get('origin_y', float())
        self.occupancy_array = array.array('i', kwargs.get('occupancy_array', []))
        self.esdf_array = array.array('d', kwargs.get('esdf_array', []))

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
        if self.width != other.width:
            return False
        if self.height != other.height:
            return False
        if self.res_x != other.res_x:
            return False
        if self.res_y != other.res_y:
            return False
        if self.origin_x != other.origin_x:
            return False
        if self.origin_y != other.origin_y:
            return False
        if self.occupancy_array != other.occupancy_array:
            return False
        if self.esdf_array != other.esdf_array:
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
    def width(self):
        """Message field 'width'."""
        return self._width

    @width.setter
    def width(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'width' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'width' field must be an integer in [-2147483648, 2147483647]"
        self._width = value

    @builtins.property
    def height(self):
        """Message field 'height'."""
        return self._height

    @height.setter
    def height(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'height' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'height' field must be an integer in [-2147483648, 2147483647]"
        self._height = value

    @builtins.property
    def res_x(self):
        """Message field 'res_x'."""
        return self._res_x

    @res_x.setter
    def res_x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'res_x' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'res_x' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._res_x = value

    @builtins.property
    def res_y(self):
        """Message field 'res_y'."""
        return self._res_y

    @res_y.setter
    def res_y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'res_y' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'res_y' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._res_y = value

    @builtins.property
    def origin_x(self):
        """Message field 'origin_x'."""
        return self._origin_x

    @origin_x.setter
    def origin_x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'origin_x' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'origin_x' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._origin_x = value

    @builtins.property
    def origin_y(self):
        """Message field 'origin_y'."""
        return self._origin_y

    @origin_y.setter
    def origin_y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'origin_y' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'origin_y' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._origin_y = value

    @builtins.property
    def occupancy_array(self):
        """Message field 'occupancy_array'."""
        return self._occupancy_array

    @occupancy_array.setter
    def occupancy_array(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'i', \
                "The 'occupancy_array' array.array() must have the type code of 'i'"
            self._occupancy_array = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, int) for v in value) and
                 all(val >= -2147483648 and val < 2147483648 for val in value)), \
                "The 'occupancy_array' field must be a set or sequence and each value of type 'int' and each integer in [-2147483648, 2147483647]"
        self._occupancy_array = array.array('i', value)

    @builtins.property
    def esdf_array(self):
        """Message field 'esdf_array'."""
        return self._esdf_array

    @esdf_array.setter
    def esdf_array(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'd', \
                "The 'esdf_array' array.array() must have the type code of 'd'"
            self._esdf_array = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'esdf_array' field must be a set or sequence and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._esdf_array = array.array('d', value)
