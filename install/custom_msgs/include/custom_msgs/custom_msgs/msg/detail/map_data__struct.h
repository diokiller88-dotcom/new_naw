// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_msgs:msg/MapData.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__MAP_DATA__STRUCT_H_
#define CUSTOM_MSGS__MSG__DETAIL__MAP_DATA__STRUCT_H_

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
// Member 'occupancy_array'
// Member 'esdf_array'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/MapData in the package custom_msgs.
typedef struct custom_msgs__msg__MapData
{
  std_msgs__msg__Header header;
  bool is_valid;
  /// 地图基础信息
  int32_t width;
  int32_t height;
  double res_x;
  double res_y;
  double origin_x;
  double origin_y;
  /// 核心地图数据 (一维展开存储)
  rosidl_runtime_c__int32__Sequence occupancy_array;
  rosidl_runtime_c__double__Sequence esdf_array;
} custom_msgs__msg__MapData;

// Struct for a sequence of custom_msgs__msg__MapData.
typedef struct custom_msgs__msg__MapData__Sequence
{
  custom_msgs__msg__MapData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_msgs__msg__MapData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_MSGS__MSG__DETAIL__MAP_DATA__STRUCT_H_
