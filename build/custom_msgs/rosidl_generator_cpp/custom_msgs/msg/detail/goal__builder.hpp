// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_msgs:msg/Goal.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__GOAL__BUILDER_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__GOAL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_msgs/msg/detail/goal__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_msgs
{

namespace msg
{

namespace builder
{

class Init_Goal_target_y
{
public:
  explicit Init_Goal_target_y(::custom_msgs::msg::Goal & msg)
  : msg_(msg)
  {}
  ::custom_msgs::msg::Goal target_y(::custom_msgs::msg::Goal::_target_y_type arg)
  {
    msg_.target_y = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_msgs::msg::Goal msg_;
};

class Init_Goal_target_x
{
public:
  Init_Goal_target_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Goal_target_y target_x(::custom_msgs::msg::Goal::_target_x_type arg)
  {
    msg_.target_x = std::move(arg);
    return Init_Goal_target_y(msg_);
  }

private:
  ::custom_msgs::msg::Goal msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_msgs::msg::Goal>()
{
  return custom_msgs::msg::builder::Init_Goal_target_x();
}

}  // namespace custom_msgs

#endif  // CUSTOM_MSGS__MSG__DETAIL__GOAL__BUILDER_HPP_
