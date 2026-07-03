// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_msgs:msg/Result.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__RESULT__STRUCT_H_
#define CUSTOM_MSGS__MSG__DETAIL__RESULT__STRUCT_H_

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

/// Struct defined in msg/Result in the package custom_msgs.
/**
  * 标准消息头，包含时间戳 (stamp) 和坐标系 (frame_id)
 */
typedef struct custom_msgs__msg__Result
{
  std_msgs__msg__Header header;
  bool is_valid;
  /// 轨迹参考点及速度
  double res_pose_x;
  double res_pose_y;
  double res_vel_x;
  double res_vel_y;
  /// 角度偏差与偏航角速度 (弧度制)
  double yaw_diff;
  double vyaw;
} custom_msgs__msg__Result;

// Struct for a sequence of custom_msgs__msg__Result.
typedef struct custom_msgs__msg__Result__Sequence
{
  custom_msgs__msg__Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_msgs__msg__Result__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_MSGS__MSG__DETAIL__RESULT__STRUCT_H_
