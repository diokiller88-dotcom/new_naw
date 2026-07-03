// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_msgs:msg/Goal.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__GOAL__STRUCT_H_
#define CUSTOM_MSGS__MSG__DETAIL__GOAL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Goal in the package custom_msgs.
typedef struct custom_msgs__msg__Goal
{
  /// 该消息包无用
  /// 终点物理坐标 (米)
  double target_x;
  double target_y;
} custom_msgs__msg__Goal;

// Struct for a sequence of custom_msgs__msg__Goal.
typedef struct custom_msgs__msg__Goal__Sequence
{
  custom_msgs__msg__Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_msgs__msg__Goal__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_MSGS__MSG__DETAIL__GOAL__STRUCT_H_
