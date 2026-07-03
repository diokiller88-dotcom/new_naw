// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_msgs:msg/ChassisInfo.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__CHASSIS_INFO__BUILDER_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__CHASSIS_INFO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_msgs/msg/detail/chassis_info__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_msgs
{

namespace msg
{

namespace builder
{

class Init_ChassisInfo_gimbal_yaw
{
public:
  explicit Init_ChassisInfo_gimbal_yaw(::custom_msgs::msg::ChassisInfo & msg)
  : msg_(msg)
  {}
  ::custom_msgs::msg::ChassisInfo gimbal_yaw(::custom_msgs::msg::ChassisInfo::_gimbal_yaw_type arg)
  {
    msg_.gimbal_yaw = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_msgs::msg::ChassisInfo msg_;
};

class Init_ChassisInfo_target_y
{
public:
  explicit Init_ChassisInfo_target_y(::custom_msgs::msg::ChassisInfo & msg)
  : msg_(msg)
  {}
  Init_ChassisInfo_gimbal_yaw target_y(::custom_msgs::msg::ChassisInfo::_target_y_type arg)
  {
    msg_.target_y = std::move(arg);
    return Init_ChassisInfo_gimbal_yaw(msg_);
  }

private:
  ::custom_msgs::msg::ChassisInfo msg_;
};

class Init_ChassisInfo_target_x
{
public:
  explicit Init_ChassisInfo_target_x(::custom_msgs::msg::ChassisInfo & msg)
  : msg_(msg)
  {}
  Init_ChassisInfo_target_y target_x(::custom_msgs::msg::ChassisInfo::_target_x_type arg)
  {
    msg_.target_x = std::move(arg);
    return Init_ChassisInfo_target_y(msg_);
  }

private:
  ::custom_msgs::msg::ChassisInfo msg_;
};

class Init_ChassisInfo_speed
{
public:
  explicit Init_ChassisInfo_speed(::custom_msgs::msg::ChassisInfo & msg)
  : msg_(msg)
  {}
  Init_ChassisInfo_target_x speed(::custom_msgs::msg::ChassisInfo::_speed_type arg)
  {
    msg_.speed = std::move(arg);
    return Init_ChassisInfo_target_x(msg_);
  }

private:
  ::custom_msgs::msg::ChassisInfo msg_;
};

class Init_ChassisInfo_trigger_target
{
public:
  explicit Init_ChassisInfo_trigger_target(::custom_msgs::msg::ChassisInfo & msg)
  : msg_(msg)
  {}
  Init_ChassisInfo_speed trigger_target(::custom_msgs::msg::ChassisInfo::_trigger_target_type arg)
  {
    msg_.trigger_target = std::move(arg);
    return Init_ChassisInfo_speed(msg_);
  }

private:
  ::custom_msgs::msg::ChassisInfo msg_;
};

class Init_ChassisInfo_trigger_relocation
{
public:
  explicit Init_ChassisInfo_trigger_relocation(::custom_msgs::msg::ChassisInfo & msg)
  : msg_(msg)
  {}
  Init_ChassisInfo_trigger_target trigger_relocation(::custom_msgs::msg::ChassisInfo::_trigger_relocation_type arg)
  {
    msg_.trigger_relocation = std::move(arg);
    return Init_ChassisInfo_trigger_target(msg_);
  }

private:
  ::custom_msgs::msg::ChassisInfo msg_;
};

class Init_ChassisInfo_is_valid
{
public:
  explicit Init_ChassisInfo_is_valid(::custom_msgs::msg::ChassisInfo & msg)
  : msg_(msg)
  {}
  Init_ChassisInfo_trigger_relocation is_valid(::custom_msgs::msg::ChassisInfo::_is_valid_type arg)
  {
    msg_.is_valid = std::move(arg);
    return Init_ChassisInfo_trigger_relocation(msg_);
  }

private:
  ::custom_msgs::msg::ChassisInfo msg_;
};

class Init_ChassisInfo_header
{
public:
  Init_ChassisInfo_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ChassisInfo_is_valid header(::custom_msgs::msg::ChassisInfo::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ChassisInfo_is_valid(msg_);
  }

private:
  ::custom_msgs::msg::ChassisInfo msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_msgs::msg::ChassisInfo>()
{
  return custom_msgs::msg::builder::Init_ChassisInfo_header();
}

}  // namespace custom_msgs

#endif  // CUSTOM_MSGS__MSG__DETAIL__CHASSIS_INFO__BUILDER_HPP_
