// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from my_custom_msgs:msg/LidarDatas.idl
// generated code does not contain a copyright notice

#ifndef MY_CUSTOM_MSGS__MSG__DETAIL__LIDAR_DATAS__STRUCT_H_
#define MY_CUSTOM_MSGS__MSG__DETAIL__LIDAR_DATAS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'data'
#include "my_custom_msgs/msg/detail/lidar_data__struct.h"

// Struct defined in msg/LidarDatas in the package my_custom_msgs.
typedef struct my_custom_msgs__msg__LidarDatas
{
  my_custom_msgs__msg__LidarData__Sequence data;
} my_custom_msgs__msg__LidarDatas;

// Struct for a sequence of my_custom_msgs__msg__LidarDatas.
typedef struct my_custom_msgs__msg__LidarDatas__Sequence
{
  my_custom_msgs__msg__LidarDatas * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} my_custom_msgs__msg__LidarDatas__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MY_CUSTOM_MSGS__MSG__DETAIL__LIDAR_DATAS__STRUCT_H_
