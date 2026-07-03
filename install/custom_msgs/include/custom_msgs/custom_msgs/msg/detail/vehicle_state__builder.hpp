// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_msgs:msg/VehicleState.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__VEHICLE_STATE__BUILDER_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__VEHICLE_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_msgs/msg/detail/vehicle_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_msgs
{

namespace msg
{

namespace builder
{

class Init_VehicleState_vel_y
{
public:
  explicit Init_VehicleState_vel_y(::custom_msgs::msg::VehicleState & msg)
  : msg_(msg)
  {}
  ::custom_msgs::msg::VehicleState vel_y(::custom_msgs::msg::VehicleState::_vel_y_type arg)
  {
    msg_.vel_y = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_msgs::msg::VehicleState msg_;
};

class Init_VehicleState_vel_x
{
public:
  explicit Init_VehicleState_vel_x(::custom_msgs::msg::VehicleState & msg)
  : msg_(msg)
  {}
  Init_VehicleState_vel_y vel_x(::custom_msgs::msg::VehicleState::_vel_x_type arg)
  {
    msg_.vel_x = std::move(arg);
    return Init_VehicleState_vel_y(msg_);
  }

private:
  ::custom_msgs::msg::VehicleState msg_;
};

class Init_VehicleState_yaw
{
public:
  explicit Init_VehicleState_yaw(::custom_msgs::msg::VehicleState & msg)
  : msg_(msg)
  {}
  Init_VehicleState_vel_x yaw(::custom_msgs::msg::VehicleState::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return Init_VehicleState_vel_x(msg_);
  }

private:
  ::custom_msgs::msg::VehicleState msg_;
};

class Init_VehicleState_pos_y
{
public:
  explicit Init_VehicleState_pos_y(::custom_msgs::msg::VehicleState & msg)
  : msg_(msg)
  {}
  Init_VehicleState_yaw pos_y(::custom_msgs::msg::VehicleState::_pos_y_type arg)
  {
    msg_.pos_y = std::move(arg);
    return Init_VehicleState_yaw(msg_);
  }

private:
  ::custom_msgs::msg::VehicleState msg_;
};

class Init_VehicleState_pos_x
{
public:
  explicit Init_VehicleState_pos_x(::custom_msgs::msg::VehicleState & msg)
  : msg_(msg)
  {}
  Init_VehicleState_pos_y pos_x(::custom_msgs::msg::VehicleState::_pos_x_type arg)
  {
    msg_.pos_x = std::move(arg);
    return Init_VehicleState_pos_y(msg_);
  }

private:
  ::custom_msgs::msg::VehicleState msg_;
};

class Init_VehicleState_is_valid
{
public:
  explicit Init_VehicleState_is_valid(::custom_msgs::msg::VehicleState & msg)
  : msg_(msg)
  {}
  Init_VehicleState_pos_x is_valid(::custom_msgs::msg::VehicleState::_is_valid_type arg)
  {
    msg_.is_valid = std::move(arg);
    return Init_VehicleState_pos_x(msg_);
  }

private:
  ::custom_msgs::msg::VehicleState msg_;
};

class Init_VehicleState_header
{
public:
  Init_VehicleState_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_VehicleState_is_valid header(::custom_msgs::msg::VehicleState::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_VehicleState_is_valid(msg_);
  }

private:
  ::custom_msgs::msg::VehicleState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_msgs::msg::VehicleState>()
{
  return custom_msgs::msg::builder::Init_VehicleState_header();
}

}  // namespace custom_msgs

#endif  // CUSTOM_MSGS__MSG__DETAIL__VEHICLE_STATE__BUILDER_HPP_
