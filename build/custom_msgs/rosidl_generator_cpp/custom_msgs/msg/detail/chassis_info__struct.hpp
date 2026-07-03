// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_msgs:msg/ChassisInfo.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__CHASSIS_INFO__STRUCT_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__CHASSIS_INFO__STRUCT_HPP_

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
# define DEPRECATED__custom_msgs__msg__ChassisInfo __attribute__((deprecated))
#else
# define DEPRECATED__custom_msgs__msg__ChassisInfo __declspec(deprecated)
#endif

namespace custom_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ChassisInfo_
{
  using Type = ChassisInfo_<ContainerAllocator>;

  explicit ChassisInfo_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->is_valid = false;
      this->trigger_relocation = false;
      this->trigger_target = false;
      this->speed = 0.0;
      this->target_x = 0.0;
      this->target_y = 0.0;
      this->gimbal_yaw = 0.0;
    }
  }

  explicit ChassisInfo_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->is_valid = false;
      this->trigger_relocation = false;
      this->trigger_target = false;
      this->speed = 0.0;
      this->target_x = 0.0;
      this->target_y = 0.0;
      this->gimbal_yaw = 0.0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _is_valid_type =
    bool;
  _is_valid_type is_valid;
  using _trigger_relocation_type =
    bool;
  _trigger_relocation_type trigger_relocation;
  using _trigger_target_type =
    bool;
  _trigger_target_type trigger_target;
  using _speed_type =
    double;
  _speed_type speed;
  using _target_x_type =
    double;
  _target_x_type target_x;
  using _target_y_type =
    double;
  _target_y_type target_y;
  using _gimbal_yaw_type =
    double;
  _gimbal_yaw_type gimbal_yaw;

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
  Type & set__trigger_relocation(
    const bool & _arg)
  {
    this->trigger_relocation = _arg;
    return *this;
  }
  Type & set__trigger_target(
    const bool & _arg)
  {
    this->trigger_target = _arg;
    return *this;
  }
  Type & set__speed(
    const double & _arg)
  {
    this->speed = _arg;
    return *this;
  }
  Type & set__target_x(
    const double & _arg)
  {
    this->target_x = _arg;
    return *this;
  }
  Type & set__target_y(
    const double & _arg)
  {
    this->target_y = _arg;
    return *this;
  }
  Type & set__gimbal_yaw(
    const double & _arg)
  {
    this->gimbal_yaw = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_msgs::msg::ChassisInfo_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_msgs::msg::ChassisInfo_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_msgs::msg::ChassisInfo_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_msgs::msg::ChassisInfo_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_msgs::msg::ChassisInfo_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_msgs::msg::ChassisInfo_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_msgs::msg::ChassisInfo_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_msgs::msg::ChassisInfo_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_msgs::msg::ChassisInfo_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_msgs::msg::ChassisInfo_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_msgs__msg__ChassisInfo
    std::shared_ptr<custom_msgs::msg::ChassisInfo_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_msgs__msg__ChassisInfo
    std::shared_ptr<custom_msgs::msg::ChassisInfo_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ChassisInfo_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->is_valid != other.is_valid) {
      return false;
    }
    if (this->trigger_relocation != other.trigger_relocation) {
      return false;
    }
    if (this->trigger_target != other.trigger_target) {
      return false;
    }
    if (this->speed != other.speed) {
      return false;
    }
    if (this->target_x != other.target_x) {
      return false;
    }
    if (this->target_y != other.target_y) {
      return false;
    }
    if (this->gimbal_yaw != other.gimbal_yaw) {
      return false;
    }
    return true;
  }
  bool operator!=(const ChassisInfo_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ChassisInfo_

// alias to use template instance with default allocator
using ChassisInfo =
  custom_msgs::msg::ChassisInfo_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace custom_msgs

#endif  // CUSTOM_MSGS__MSG__DETAIL__CHASSIS_INFO__STRUCT_HPP_
