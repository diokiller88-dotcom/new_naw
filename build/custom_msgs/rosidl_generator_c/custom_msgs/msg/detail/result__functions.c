// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from custom_msgs:msg/Result.idl
// generated code does not contain a copyright notice
#include "custom_msgs/msg/detail/result__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
custom_msgs__msg__Result__init(custom_msgs__msg__Result * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    custom_msgs__msg__Result__fini(msg);
    return false;
  }
  // is_valid
  // res_pose_x
  // res_pose_y
  // res_vel_x
  // res_vel_y
  // yaw_diff
  // vyaw
  return true;
}

void
custom_msgs__msg__Result__fini(custom_msgs__msg__Result * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // is_valid
  // res_pose_x
  // res_pose_y
  // res_vel_x
  // res_vel_y
  // yaw_diff
  // vyaw
}

bool
custom_msgs__msg__Result__are_equal(const custom_msgs__msg__Result * lhs, const custom_msgs__msg__Result * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // is_valid
  if (lhs->is_valid != rhs->is_valid) {
    return false;
  }
  // res_pose_x
  if (lhs->res_pose_x != rhs->res_pose_x) {
    return false;
  }
  // res_pose_y
  if (lhs->res_pose_y != rhs->res_pose_y) {
    return false;
  }
  // res_vel_x
  if (lhs->res_vel_x != rhs->res_vel_x) {
    return false;
  }
  // res_vel_y
  if (lhs->res_vel_y != rhs->res_vel_y) {
    return false;
  }
  // yaw_diff
  if (lhs->yaw_diff != rhs->yaw_diff) {
    return false;
  }
  // vyaw
  if (lhs->vyaw != rhs->vyaw) {
    return false;
  }
  return true;
}

bool
custom_msgs__msg__Result__copy(
  const custom_msgs__msg__Result * input,
  custom_msgs__msg__Result * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // is_valid
  output->is_valid = input->is_valid;
  // res_pose_x
  output->res_pose_x = input->res_pose_x;
  // res_pose_y
  output->res_pose_y = input->res_pose_y;
  // res_vel_x
  output->res_vel_x = input->res_vel_x;
  // res_vel_y
  output->res_vel_y = input->res_vel_y;
  // yaw_diff
  output->yaw_diff = input->yaw_diff;
  // vyaw
  output->vyaw = input->vyaw;
  return true;
}

custom_msgs__msg__Result *
custom_msgs__msg__Result__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_msgs__msg__Result * msg = (custom_msgs__msg__Result *)allocator.allocate(sizeof(custom_msgs__msg__Result), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(custom_msgs__msg__Result));
  bool success = custom_msgs__msg__Result__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
custom_msgs__msg__Result__destroy(custom_msgs__msg__Result * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    custom_msgs__msg__Result__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
custom_msgs__msg__Result__Sequence__init(custom_msgs__msg__Result__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_msgs__msg__Result * data = NULL;

  if (size) {
    data = (custom_msgs__msg__Result *)allocator.zero_allocate(size, sizeof(custom_msgs__msg__Result), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = custom_msgs__msg__Result__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        custom_msgs__msg__Result__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
custom_msgs__msg__Result__Sequence__fini(custom_msgs__msg__Result__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      custom_msgs__msg__Result__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

custom_msgs__msg__Result__Sequence *
custom_msgs__msg__Result__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_msgs__msg__Result__Sequence * array = (custom_msgs__msg__Result__Sequence *)allocator.allocate(sizeof(custom_msgs__msg__Result__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = custom_msgs__msg__Result__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
custom_msgs__msg__Result__Sequence__destroy(custom_msgs__msg__Result__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    custom_msgs__msg__Result__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
custom_msgs__msg__Result__Sequence__are_equal(const custom_msgs__msg__Result__Sequence * lhs, const custom_msgs__msg__Result__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!custom_msgs__msg__Result__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
custom_msgs__msg__Result__Sequence__copy(
  const custom_msgs__msg__Result__Sequence * input,
  custom_msgs__msg__Result__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(custom_msgs__msg__Result);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    custom_msgs__msg__Result * data =
      (custom_msgs__msg__Result *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!custom_msgs__msg__Result__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          custom_msgs__msg__Result__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!custom_msgs__msg__Result__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
