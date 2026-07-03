// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from custom_msgs:msg/MapData.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__MAP_DATA__TRAITS_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__MAP_DATA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "custom_msgs/msg/detail/map_data__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace custom_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const MapData & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: is_valid
  {
    out << "is_valid: ";
    rosidl_generator_traits::value_to_yaml(msg.is_valid, out);
    out << ", ";
  }

  // member: width
  {
    out << "width: ";
    rosidl_generator_traits::value_to_yaml(msg.width, out);
    out << ", ";
  }

  // member: height
  {
    out << "height: ";
    rosidl_generator_traits::value_to_yaml(msg.height, out);
    out << ", ";
  }

  // member: res_x
  {
    out << "res_x: ";
    rosidl_generator_traits::value_to_yaml(msg.res_x, out);
    out << ", ";
  }

  // member: res_y
  {
    out << "res_y: ";
    rosidl_generator_traits::value_to_yaml(msg.res_y, out);
    out << ", ";
  }

  // member: origin_x
  {
    out << "origin_x: ";
    rosidl_generator_traits::value_to_yaml(msg.origin_x, out);
    out << ", ";
  }

  // member: origin_y
  {
    out << "origin_y: ";
    rosidl_generator_traits::value_to_yaml(msg.origin_y, out);
    out << ", ";
  }

  // member: occupancy_array
  {
    if (msg.occupancy_array.size() == 0) {
      out << "occupancy_array: []";
    } else {
      out << "occupancy_array: [";
      size_t pending_items = msg.occupancy_array.size();
      for (auto item : msg.occupancy_array) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: esdf_array
  {
    if (msg.esdf_array.size() == 0) {
      out << "esdf_array: []";
    } else {
      out << "esdf_array: [";
      size_t pending_items = msg.esdf_array.size();
      for (auto item : msg.esdf_array) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MapData & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: is_valid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_valid: ";
    rosidl_generator_traits::value_to_yaml(msg.is_valid, out);
    out << "\n";
  }

  // member: width
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "width: ";
    rosidl_generator_traits::value_to_yaml(msg.width, out);
    out << "\n";
  }

  // member: height
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "height: ";
    rosidl_generator_traits::value_to_yaml(msg.height, out);
    out << "\n";
  }

  // member: res_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "res_x: ";
    rosidl_generator_traits::value_to_yaml(msg.res_x, out);
    out << "\n";
  }

  // member: res_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "res_y: ";
    rosidl_generator_traits::value_to_yaml(msg.res_y, out);
    out << "\n";
  }

  // member: origin_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "origin_x: ";
    rosidl_generator_traits::value_to_yaml(msg.origin_x, out);
    out << "\n";
  }

  // member: origin_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "origin_y: ";
    rosidl_generator_traits::value_to_yaml(msg.origin_y, out);
    out << "\n";
  }

  // member: occupancy_array
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.occupancy_array.size() == 0) {
      out << "occupancy_array: []\n";
    } else {
      out << "occupancy_array:\n";
      for (auto item : msg.occupancy_array) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: esdf_array
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.esdf_array.size() == 0) {
      out << "esdf_array: []\n";
    } else {
      out << "esdf_array:\n";
      for (auto item : msg.esdf_array) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MapData & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace custom_msgs

namespace rosidl_generator_traits
{

[[deprecated("use custom_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const custom_msgs::msg::MapData & msg,
  std::ostream & out, size_t indentation = 0)
{
  custom_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use custom_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const custom_msgs::msg::MapData & msg)
{
  return custom_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<custom_msgs::msg::MapData>()
{
  return "custom_msgs::msg::MapData";
}

template<>
inline const char * name<custom_msgs::msg::MapData>()
{
  return "custom_msgs/msg/MapData";
}

template<>
struct has_fixed_size<custom_msgs::msg::MapData>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<custom_msgs::msg::MapData>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<custom_msgs::msg::MapData>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CUSTOM_MSGS__MSG__DETAIL__MAP_DATA__TRAITS_HPP_
