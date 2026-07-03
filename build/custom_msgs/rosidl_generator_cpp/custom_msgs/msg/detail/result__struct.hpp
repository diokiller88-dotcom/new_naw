// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_msgs:msg/Result.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__RESULT__STRUCT_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__RESULT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__custom_msgs__msg__Result __attribute__((deprecated))
#else
# define DEPRECATED__custom_msgs__msg__Result __declspec(deprecated)
#endif

namespace custom_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Result_
{
  using Type = Result_<ContainerAllocator>;

  explicit Result_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->is_valid = false;
      this->res_pose_x = 0.0;
      this->res_pose_y = 0.0;
      this->res_vel_x = 0.0;
      this->res_vel_y = 0.0;
      this->yaw_diff = 0.0;
      this->vyaw = 0.0;
    }
  }

  explicit Result_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->is_valid = false;
      this->res_pose_x = 0.0;
      this->res_pose_y = 0.0;
      this->res_vel_x = 0.0;
      this->res_vel_y = 0.0;
      this->yaw_diff = 0.0;
      this->vyaw = 0.0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _is_valid_type =
    bool;
  _is_valid_type is_valid;
  using _res_pose_x_type =
    double;
  _res_pose_x_type res_pose_x;
  using _res_pose_y_type =
    double;
  _res_pose_y_type res_pose_y;
  using _res_vel_x_type =
    double;
  _res_vel_x_type res_vel_x;
  using _res_vel_y_type =
    double;
  _res_vel_y_type res_vel_y;
  using _yaw_diff_type =
    double;
  _yaw_diff_type yaw_diff;
  using _vyaw_type =
    double;
  _vyaw_type vyaw;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__is_valid(
    const bool & _arg)
  {
    this->is_valid = _arg;
    return *this;
  }
  Type & set__res_pose_x(
    const double & _arg)
  {
    this->res_pose_x = _arg;
    return *this;
  }
  Type & set__res_pose_y(
    const double & _arg)
  {
    this->res_pose_y = _arg;
    return *this;
  }
  Type & set__res_vel_x(
    const double & _arg)
  {
    this->res_vel_x = _arg;
    return *this;
  }
  Type & set__res_vel_y(
    const double & _arg)
  {
    this->res_vel_y = _arg;
    return *this;
  }
  Type & set__yaw_diff(
    const double & _arg)
  {
    this->yaw_diff = _arg;
    return *this;
  }
  Type & set__vyaw(
    const double & _arg)
  {
    this->vyaw = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_msgs::msg::Result_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_msgs::msg::Result_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_msgs::msg::Result_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_msgs::msg::Result_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_msgs::msg::Result_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_msgs::msg::Result_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_msgs::msg::Result_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_msgs::msg::Result_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_msgs::msg::Result_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_msgs::msg::Result_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_msgs__msg__Result
    std::shared_ptr<custom_msgs::msg::Result_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_msgs__msg__Result
    std::shared_ptr<custom_msgs::msg::Result_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Result_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->is_valid != other.is_valid) {
      return false;
    }
    if (this->res_pose_x != other.res_pose_x) {
      return false;
    }
    if (this->res_pose_y != other.res_pose_y) {
      return false;
    }
    if (this->res_vel_x != other.res_vel_x) {
      return false;
    }
    if (this->res_vel_y != other.res_vel_y) {
      return false;
    }
    if (this->yaw_diff != other.yaw_diff) {
      return false;
    }
    if (this->vyaw != other.vyaw) {
      return false;
    }
    return true;
  }
  bool operator!=(const Result_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Result_

// alias to use template instance with default allocator
using Result =
  custom_msgs::msg::Result_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace custom_msgs

#endif  // CUSTOM_MSGS__MSG__DETAIL__RESULT__STRUCT_HPP_
