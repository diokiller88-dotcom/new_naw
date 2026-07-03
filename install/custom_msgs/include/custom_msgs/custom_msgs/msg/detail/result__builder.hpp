// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_msgs:msg/Result.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__RESULT__BUILDER_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__RESULT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_msgs/msg/detail/result__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_msgs
{

namespace msg
{

namespace builder
{

class Init_Result_vyaw
{
public:
  explicit Init_Result_vyaw(::custom_msgs::msg::Result & msg)
  : msg_(msg)
  {}
  ::custom_msgs::msg::Result vyaw(::custom_msgs::msg::Result::_vyaw_type arg)
  {
    msg_.vyaw = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_msgs::msg::Result msg_;
};

class Init_Result_yaw_diff
{
public:
  explicit Init_Result_yaw_diff(::custom_msgs::msg::Result & msg)
  : msg_(msg)
  {}
  Init_Result_vyaw yaw_diff(::custom_msgs::msg::Result::_yaw_diff_type arg)
  {
    msg_.yaw_diff = std::move(arg);
    return Init_Result_vyaw(msg_);
  }

private:
  ::custom_msgs::msg::Result msg_;
};

class Init_Result_res_vel_y
{
public:
  explicit Init_Result_res_vel_y(::custom_msgs::msg::Result & msg)
  : msg_(msg)
  {}
  Init_Result_yaw_diff res_vel_y(::custom_msgs::msg::Result::_res_vel_y_type arg)
  {
    msg_.res_vel_y = std::move(arg);
    return Init_Result_yaw_diff(msg_);
  }

private:
  ::custom_msgs::msg::Result msg_;
};

class Init_Result_res_vel_x
{
public:
  explicit Init_Result_res_vel_x(::custom_msgs::msg::Result & msg)
  : msg_(msg)
  {}
  Init_Result_res_vel_y res_vel_x(::custom_msgs::msg::Result::_res_vel_x_type arg)
  {
    msg_.res_vel_x = std::move(arg);
    return Init_Result_res_vel_y(msg_);
  }

private:
  ::custom_msgs::msg::Result msg_;
};

class Init_Result_res_pose_y
{
public:
  explicit Init_Result_res_pose_y(::custom_msgs::msg::Result & msg)
  : msg_(msg)
  {}
  Init_Result_res_vel_x res_pose_y(::custom_msgs::msg::Result::_res_pose_y_type arg)
  {
    msg_.res_pose_y = std::move(arg);
    return Init_Result_res_vel_x(msg_);
  }

private:
  ::custom_msgs::msg::Result msg_;
};

class Init_Result_res_pose_x
{
public:
  explicit Init_Result_res_pose_x(::custom_msgs::msg::Result & msg)
  : msg_(msg)
  {}
  Init_Result_res_pose_y res_pose_x(::custom_msgs::msg::Result::_res_pose_x_type arg)
  {
    msg_.res_pose_x = std::move(arg);
    return Init_Result_res_pose_y(msg_);
  }

private:
  ::custom_msgs::msg::Result msg_;
};

class Init_Result_is_valid
{
public:
  explicit Init_Result_is_valid(::custom_msgs::msg::Result & msg)
  : msg_(msg)
  {}
  Init_Result_res_pose_x is_valid(::custom_msgs::msg::Result::_is_valid_type arg)
  {
    msg_.is_valid = std::move(arg);
    return Init_Result_res_pose_x(msg_);
  }

private:
  ::custom_msgs::msg::Result msg_;
};

class Init_Result_header
{
public:
  Init_Result_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Result_is_valid header(::custom_msgs::msg::Result::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Result_is_valid(msg_);
  }

private:
  ::custom_msgs::msg::Result msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_msgs::msg::Result>()
{
  return custom_msgs::msg::builder::Init_Result_header();
}

}  // namespace custom_msgs

#endif  // CUSTOM_MSGS__MSG__DETAIL__RESULT__BUILDER_HPP_
