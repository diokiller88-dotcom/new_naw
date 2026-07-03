// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_msgs:msg/MapData.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__MAP_DATA__STRUCT_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__MAP_DATA__STRUCT_HPP_

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
# define DEPRECATED__custom_msgs__msg__MapData __attribute__((deprecated))
#else
# define DEPRECATED__custom_msgs__msg__MapData __declspec(deprecated)
#endif

namespace custom_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MapData_
{
  using Type = MapData_<ContainerAllocator>;

  explicit MapData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->is_valid = false;
      this->width = 0l;
      this->height = 0l;
      this->res_x = 0.0;
      this->res_y = 0.0;
      this->origin_x = 0.0;
      this->origin_y = 0.0;
    }
  }

  explicit MapData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->is_valid = false;
      this->width = 0l;
      this->height = 0l;
      this->res_x = 0.0;
      this->res_y = 0.0;
      this->origin_x = 0.0;
      this->origin_y = 0.0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _is_valid_type =
    bool;
  _is_valid_type is_valid;
  using _width_type =
    int32_t;
  _width_type width;
  using _height_type =
    int32_t;
  _height_type height;
  using _res_x_type =
    double;
  _res_x_type res_x;
  using _res_y_type =
    double;
  _res_y_type res_y;
  using _origin_x_type =
    double;
  _origin_x_type origin_x;
  using _origin_y_type =
    double;
  _origin_y_type origin_y;
  using _occupancy_array_type =
    std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>>;
  _occupancy_array_type occupancy_array;
  using _esdf_array_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _esdf_array_type esdf_array;

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
  Type & set__width(
    const int32_t & _arg)
  {
    this->width = _arg;
    return *this;
  }
  Type & set__height(
    const int32_t & _arg)
  {
    this->height = _arg;
    return *this;
  }
  Type & set__res_x(
    const double & _arg)
  {
    this->res_x = _arg;
    return *this;
  }
  Type & set__res_y(
    const double & _arg)
  {
    this->res_y = _arg;
    return *this;
  }
  Type & set__origin_x(
    const double & _arg)
  {
    this->origin_x = _arg;
    return *this;
  }
  Type & set__origin_y(
    const double & _arg)
  {
    this->origin_y = _arg;
    return *this;
  }
  Type & set__occupancy_array(
    const std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>> & _arg)
  {
    this->occupancy_array = _arg;
    return *this;
  }
  Type & set__esdf_array(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->esdf_array = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_msgs::msg::MapData_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_msgs::msg::MapData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_msgs::msg::MapData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_msgs::msg::MapData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_msgs::msg::MapData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_msgs::msg::MapData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_msgs::msg::MapData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_msgs::msg::MapData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_msgs::msg::MapData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_msgs::msg::MapData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_msgs__msg__MapData
    std::shared_ptr<custom_msgs::msg::MapData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_msgs__msg__MapData
    std::shared_ptr<custom_msgs::msg::MapData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MapData_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->is_valid != other.is_valid) {
      return false;
    }
    if (this->width != other.width) {
      return false;
    }
    if (this->height != other.height) {
      return false;
    }
    if (this->res_x != other.res_x) {
      return false;
    }
    if (this->res_y != other.res_y) {
      return false;
    }
    if (this->origin_x != other.origin_x) {
      return false;
    }
    if (this->origin_y != other.origin_y) {
      return false;
    }
    if (this->occupancy_array != other.occupancy_array) {
      return false;
    }
    if (this->esdf_array != other.esdf_array) {
      return false;
    }
    return true;
  }
  bool operator!=(const MapData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MapData_

// alias to use template instance with default allocator
using MapData =
  custom_msgs::msg::MapData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace custom_msgs

#endif  // CUSTOM_MSGS__MSG__DETAIL__MAP_DATA__STRUCT_HPP_
