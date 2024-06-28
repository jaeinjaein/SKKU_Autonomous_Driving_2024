// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from my_custom_msgs:msg/LidarDatas.idl
// generated code does not contain a copyright notice
#include "my_custom_msgs/msg/detail/lidar_datas__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `data`
#include "my_custom_msgs/msg/detail/lidar_data__functions.h"

bool
my_custom_msgs__msg__LidarDatas__init(my_custom_msgs__msg__LidarDatas * msg)
{
  if (!msg) {
    return false;
  }
  // data
  if (!my_custom_msgs__msg__LidarData__Sequence__init(&msg->data, 0)) {
    my_custom_msgs__msg__LidarDatas__fini(msg);
    return false;
  }
  return true;
}

void
my_custom_msgs__msg__LidarDatas__fini(my_custom_msgs__msg__LidarDatas * msg)
{
  if (!msg) {
    return;
  }
  // data
  my_custom_msgs__msg__LidarData__Sequence__fini(&msg->data);
}

bool
my_custom_msgs__msg__LidarDatas__are_equal(const my_custom_msgs__msg__LidarDatas * lhs, const my_custom_msgs__msg__LidarDatas * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // data
  if (!my_custom_msgs__msg__LidarData__Sequence__are_equal(
      &(lhs->data), &(rhs->data)))
  {
    return false;
  }
  return true;
}

bool
my_custom_msgs__msg__LidarDatas__copy(
  const my_custom_msgs__msg__LidarDatas * input,
  my_custom_msgs__msg__LidarDatas * output)
{
  if (!input || !output) {
    return false;
  }
  // data
  if (!my_custom_msgs__msg__LidarData__Sequence__copy(
      &(input->data), &(output->data)))
  {
    return false;
  }
  return true;
}

my_custom_msgs__msg__LidarDatas *
my_custom_msgs__msg__LidarDatas__create()
{
  my_custom_msgs__msg__LidarDatas * msg = (my_custom_msgs__msg__LidarDatas *)malloc(sizeof(my_custom_msgs__msg__LidarDatas));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(my_custom_msgs__msg__LidarDatas));
  bool success = my_custom_msgs__msg__LidarDatas__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
my_custom_msgs__msg__LidarDatas__destroy(my_custom_msgs__msg__LidarDatas * msg)
{
  if (msg) {
    my_custom_msgs__msg__LidarDatas__fini(msg);
  }
  free(msg);
}


bool
my_custom_msgs__msg__LidarDatas__Sequence__init(my_custom_msgs__msg__LidarDatas__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  my_custom_msgs__msg__LidarDatas * data = NULL;
  if (size) {
    data = (my_custom_msgs__msg__LidarDatas *)calloc(size, sizeof(my_custom_msgs__msg__LidarDatas));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = my_custom_msgs__msg__LidarDatas__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        my_custom_msgs__msg__LidarDatas__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
my_custom_msgs__msg__LidarDatas__Sequence__fini(my_custom_msgs__msg__LidarDatas__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      my_custom_msgs__msg__LidarDatas__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

my_custom_msgs__msg__LidarDatas__Sequence *
my_custom_msgs__msg__LidarDatas__Sequence__create(size_t size)
{
  my_custom_msgs__msg__LidarDatas__Sequence * array = (my_custom_msgs__msg__LidarDatas__Sequence *)malloc(sizeof(my_custom_msgs__msg__LidarDatas__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = my_custom_msgs__msg__LidarDatas__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
my_custom_msgs__msg__LidarDatas__Sequence__destroy(my_custom_msgs__msg__LidarDatas__Sequence * array)
{
  if (array) {
    my_custom_msgs__msg__LidarDatas__Sequence__fini(array);
  }
  free(array);
}

bool
my_custom_msgs__msg__LidarDatas__Sequence__are_equal(const my_custom_msgs__msg__LidarDatas__Sequence * lhs, const my_custom_msgs__msg__LidarDatas__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!my_custom_msgs__msg__LidarDatas__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
my_custom_msgs__msg__LidarDatas__Sequence__copy(
  const my_custom_msgs__msg__LidarDatas__Sequence * input,
  my_custom_msgs__msg__LidarDatas__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(my_custom_msgs__msg__LidarDatas);
    my_custom_msgs__msg__LidarDatas * data =
      (my_custom_msgs__msg__LidarDatas *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!my_custom_msgs__msg__LidarDatas__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          my_custom_msgs__msg__LidarDatas__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!my_custom_msgs__msg__LidarDatas__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
