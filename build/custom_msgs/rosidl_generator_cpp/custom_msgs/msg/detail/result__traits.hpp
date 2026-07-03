// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from custom_msgs:msg/Result.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__RESULT__TRAITS_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__RESULT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "custom_msgs/msg/detail/result__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace custom_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Result & msg,
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

  // member: res_pose_x
  {
    out << "res_pose_x: ";
    rosidl_generator_traits::value_to_yaml(msg.res_pose_x, out);
    out << ", ";
  }

  // member: res_pose_y
  {
    out << "res_pose_y: ";
    rosidl_generator_traits::value_to_yaml(msg.res_pose_y, out);
    out << ", ";
  }

  // member: res_vel_x
  {
    out << "res_vel_x: ";
    rosidl_generator_traits::value_to_yaml(msg.res_vel_x, out);
    out << ", ";
  }

  // member: res_vel_y
  {
    out << "res_vel_y: ";
    rosidl_generator_traits::value_to_yaml(msg.res_vel_y, out);
    out << ", ";
  }

  // member: yaw_diff
  {
    out << "yaw_diff: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw_diff, out);
    out << ", ";
  }

  // member: vyaw
  {
    out << "vyaw: ";
    rosidl_generator_traits::value_to_yaml(msg.vyaw, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Result & msg,
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

  // member: res_pose_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "res_pose_x: ";
    rosidl_generator_traits::value_to_yaml(msg.res_pose_x, out);
    out << "\n";
  }

  // member: res_pose_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "res_pose_y: ";
    rosidl_generator_traits::value_to_yaml(msg.res_pose_y, out);
    out << "\n";
  }

  // member: res_vel_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "res_vel_x: ";
    rosidl_generator_traits::value_to_yaml(msg.res_vel_x, out);
    out << "\n";
  }

  // member: res_vel_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "res_vel_y: ";
    rosidl_generator_traits::value_to_yaml(msg.res_vel_y, out);
    out << "\n";
  }

  // member: yaw_diff
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw_diff: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw_diff, out);
    out << "\n";
  }

  // member: vyaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vyaw: ";
    rosidl_generator_traits::value_to_yaml(msg.vyaw, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Result & msg, bool use_flow_style = false)
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
  const custom_msgs::msg::Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  custom_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use custom_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const custom_msgs::msg::Result & msg)
{
  return custom_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<custom_msgs::msg::Result>()
{
  return "custom_msgs::msg::Result";
}

template<>
inline const char * name<custom_msgs::msg::Result>()
{
  return "custom_msgs/msg/Result";
}

template<>
struct has_fixed_size<custom_msgs::msg::Result>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<custom_msgs::msg::Result>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<custom_msgs::msg::Result>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CUSTOM_MSGS__MSG__DETAIL__RESULT__TRAITS_HPP_
