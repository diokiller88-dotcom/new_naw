// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_msgs:msg/MapData.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__MAP_DATA__BUILDER_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__MAP_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_msgs/msg/detail/map_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_msgs
{

namespace msg
{

namespace builder
{

class Init_MapData_esdf_array
{
public:
  explicit Init_MapData_esdf_array(::custom_msgs::msg::MapData & msg)
  : msg_(msg)
  {}
  ::custom_msgs::msg::MapData esdf_array(::custom_msgs::msg::MapData::_esdf_array_type arg)
  {
    msg_.esdf_array = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_msgs::msg::MapData msg_;
};

class Init_MapData_occupancy_array
{
public:
  explicit Init_MapData_occupancy_array(::custom_msgs::msg::MapData & msg)
  : msg_(msg)
  {}
  Init_MapData_esdf_array occupancy_array(::custom_msgs::msg::MapData::_occupancy_array_type arg)
  {
    msg_.occupancy_array = std::move(arg);
    return Init_MapData_esdf_array(msg_);
  }

private:
  ::custom_msgs::msg::MapData msg_;
};

class Init_MapData_origin_y
{
public:
  explicit Init_MapData_origin_y(::custom_msgs::msg::MapData & msg)
  : msg_(msg)
  {}
  Init_MapData_occupancy_array origin_y(::custom_msgs::msg::MapData::_origin_y_type arg)
  {
    msg_.origin_y = std::move(arg);
    return Init_MapData_occupancy_array(msg_);
  }

private:
  ::custom_msgs::msg::MapData msg_;
};

class Init_MapData_origin_x
{
public:
  explicit Init_MapData_origin_x(::custom_msgs::msg::MapData & msg)
  : msg_(msg)
  {}
  Init_MapData_origin_y origin_x(::custom_msgs::msg::MapData::_origin_x_type arg)
  {
    msg_.origin_x = std::move(arg);
    return Init_MapData_origin_y(msg_);
  }

private:
  ::custom_msgs::msg::MapData msg_;
};

class Init_MapData_res_y
{
public:
  explicit Init_MapData_res_y(::custom_msgs::msg::MapData & msg)
  : msg_(msg)
  {}
  Init_MapData_origin_x res_y(::custom_msgs::msg::MapData::_res_y_type arg)
  {
    msg_.res_y = std::move(arg);
    return Init_MapData_origin_x(msg_);
  }

private:
  ::custom_msgs::msg::MapData msg_;
};

class Init_MapData_res_x
{
public:
  explicit Init_MapData_res_x(::custom_msgs::msg::MapData & msg)
  : msg_(msg)
  {}
  Init_MapData_res_y res_x(::custom_msgs::msg::MapData::_res_x_type arg)
  {
    msg_.res_x = std::move(arg);
    return Init_MapData_res_y(msg_);
  }

private:
  ::custom_msgs::msg::MapData msg_;
};

class Init_MapData_height
{
public:
  explicit Init_MapData_height(::custom_msgs::msg::MapData & msg)
  : msg_(msg)
  {}
  Init_MapData_res_x height(::custom_msgs::msg::MapData::_height_type arg)
  {
    msg_.height = std::move(arg);
    return Init_MapData_res_x(msg_);
  }

private:
  ::custom_msgs::msg::MapData msg_;
};

class Init_MapData_width
{
public:
  explicit Init_MapData_width(::custom_msgs::msg::MapData & msg)
  : msg_(msg)
  {}
  Init_MapData_height width(::custom_msgs::msg::MapData::_width_type arg)
  {
    msg_.width = std::move(arg);
    return Init_MapData_height(msg_);
  }

private:
  ::custom_msgs::msg::MapData msg_;
};

class Init_MapData_is_valid
{
public:
  explicit Init_MapData_is_valid(::custom_msgs::msg::MapData & msg)
  : msg_(msg)
  {}
  Init_MapData_width is_valid(::custom_msgs::msg::MapData::_is_valid_type arg)
  {
    msg_.is_valid = std::move(arg);
    return Init_MapData_width(msg_);
  }

private:
  ::custom_msgs::msg::MapData msg_;
};

class Init_MapData_header
{
public:
  Init_MapData_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MapData_is_valid header(::custom_msgs::msg::MapData::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MapData_is_valid(msg_);
  }

private:
  ::custom_msgs::msg::MapData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_msgs::msg::MapData>()
{
  return custom_msgs::msg::builder::Init_MapData_header();
}

}  // namespace custom_msgs

#endif  // CUSTOM_MSGS__MSG__DETAIL__MAP_DATA__BUILDER_HPP_
