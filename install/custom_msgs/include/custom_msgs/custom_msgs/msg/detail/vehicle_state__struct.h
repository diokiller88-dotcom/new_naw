// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_msgs:msg/VehicleState.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__VEHICLE_STATE__STRUCT_H_
#define CUSTOM_MSGS__MSG__DETAIL__VEHICLE_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/VehicleState in the package custom_msgs.
typedef struct custom_msgs__msg__VehicleState
{
  std_msgs__msg__Header header;
  bool is_valid;
  /// 当前车体物理坐标 (米) - 地图坐标系
  double pos_x;
  double pos_y;
  /// 当前底盘在地图坐标系下的绝对航向角 (弧度)
  double yaw;
  /// 当前车体线速度 (米/秒) - 地图坐标系
  double vel_x;
  double vel_y;
} custom_msgs__msg__VehicleState;

// Struct for a sequence of custom_msgs__msg__VehicleState.
typedef struct custom_msgs__msg__VehicleState__Sequence
{
  custom_msgs__msg__VehicleState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_msgs__msg__VehicleState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_MSGS__MSG__DETAIL__VEHICLE_STATE__STRUCT_H_
