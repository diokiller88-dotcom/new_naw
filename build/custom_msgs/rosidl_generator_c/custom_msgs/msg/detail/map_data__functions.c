// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from custom_msgs:msg/MapData.idl
// generated code does not contain a copyright notice
#include "custom_msgs/msg/detail/map_data__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `occupancy_array`
// Member `esdf_array`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
custom_msgs__msg__MapData__init(custom_msgs__msg__MapData * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    custom_msgs__msg__MapData__fini(msg);
    return false;
  }
  // is_valid
  // width
  // height
  // res_x
  // res_y
  // origin_x
  // origin_y
  // occupancy_array
  if (!rosidl_runtime_c__int32__Sequence__init(&msg->occupancy_array, 0)) {
    custom_msgs__msg__MapData__fini(msg);
    return false;
  }
  // esdf_array
  if (!rosidl_runtime_c__double__Sequence__init(&msg->esdf_array, 0)) {
    custom_msgs__msg__MapData__fini(msg);
    return false;
  }
  return true;
}

void
custom_msgs__msg__MapData__fini(custom_msgs__msg__MapData * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // is_valid
  // width
  // height
  // res_x
  // res_y
  // origin_x
  // origin_y
  // occupancy_array
  rosidl_runtime_c__int32__Sequence__fini(&msg->occupancy_array);
  // esdf_array
  rosidl_runtime_c__double__Sequence__fini(&msg->esdf_array);
}

bool
custom_msgs__msg__MapData__are_equal(const custom_msgs__msg__MapData * lhs, const custom_msgs__msg__MapData * rhs)
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
  // width
  if (lhs->width != rhs->width) {
    return false;
  }
  // height
  if (lhs->height != rhs->height) {
    return false;
  }
  // res_x
  if (lhs->res_x != rhs->res_x) {
    return false;
  }
  // res_y
  if (lhs->res_y != rhs->res_y) {
    return false;
  }
  // origin_x
  if (lhs->origin_x != rhs->origin_x) {
    return false;
  }
  // origin_y
  if (lhs->origin_y != rhs->origin_y) {
    return false;
  }
  // occupancy_array
  if (!rosidl_runtime_c__int32__Sequence__are_equal(
      &(lhs->occupancy_array), &(rhs->occupancy_array)))
  {
    return false;
  }
  // esdf_array
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->esdf_array), &(rhs->esdf_array)))
  {
    return false;
  }
  return true;
}

bool
custom_msgs__msg__MapData__copy(
  const custom_msgs__msg__MapData * input,
  custom_msgs__msg__MapData * output)
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
  // width
  output->width = input->width;
  // height
  output->height = input->height;
  // res_x
  output->res_x = input->res_x;
  // res_y
  output->res_y = input->res_y;
  // origin_x
  output->origin_x = input->origin_x;
  // origin_y
  output->origin_y = input->origin_y;
  // occupancy_array
  if (!rosidl_runtime_c__int32__Sequence__copy(
      &(input->occupancy_array), &(output->occupancy_array)))
  {
    return false;
  }
  // esdf_array
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->esdf_array), &(output->esdf_array)))
  {
    return false;
  }
  return true;
}

custom_msgs__msg__MapData *
custom_msgs__msg__MapData__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_msgs__msg__MapData * msg = (custom_msgs__msg__MapData *)allocator.allocate(sizeof(custom_msgs__msg__MapData), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(custom_msgs__msg__MapData));
  bool success = custom_msgs__msg__MapData__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
custom_msgs__msg__MapData__destroy(custom_msgs__msg__MapData * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    custom_msgs__msg__MapData__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
custom_msgs__msg__MapData__Sequence__init(custom_msgs__msg__MapData__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_msgs__msg__MapData * data = NULL;

  if (size) {
    data = (custom_msgs__msg__MapData *)allocator.zero_allocate(size, sizeof(custom_msgs__msg__MapData), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = custom_msgs__msg__MapData__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        custom_msgs__msg__MapData__fini(&data[i - 1]);
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
custom_msgs__msg__MapData__Sequence__fini(custom_msgs__msg__MapData__Sequence * array)
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
      custom_msgs__msg__MapData__fini(&array->data[i]);
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

custom_msgs__msg__MapData__Sequence *
custom_msgs__msg__MapData__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_msgs__msg__MapData__Sequence * array = (custom_msgs__msg__MapData__Sequence *)allocator.allocate(sizeof(custom_msgs__msg__MapData__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = custom_msgs__msg__MapData__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
custom_msgs__msg__MapData__Sequence__destroy(custom_msgs__msg__MapData__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    custom_msgs__msg__MapData__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
custom_msgs__msg__MapData__Sequence__are_equal(const custom_msgs__msg__MapData__Sequence * lhs, const custom_msgs__msg__MapData__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!custom_msgs__msg__MapData__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
custom_msgs__msg__MapData__Sequence__copy(
  const custom_msgs__msg__MapData__Sequence * input,
  custom_msgs__msg__MapData__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(custom_msgs__msg__MapData);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    custom_msgs__msg__MapData * data =
      (custom_msgs__msg__MapData *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!custom_msgs__msg__MapData__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          custom_msgs__msg__MapData__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!custom_msgs__msg__MapData__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
