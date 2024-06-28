// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from my_custom_msgs:msg/LidarData.idl
// generated code does not contain a copyright notice

#ifndef MY_CUSTOM_MSGS__MSG__DETAIL__LIDAR_DATA__STRUCT_H_
#define MY_CUSTOM_MSGS__MSG__DETAIL__LIDAR_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/LidarData in the package my_custom_msgs.
typedef struct my_custom_msgs__msg__LidarData
{
  float angle;
  float distance;
} my_custom_msgs__msg__LidarData;

// Struct for a sequence of my_custom_msgs__msg__LidarData.
typedef struct my_custom_msgs__msg__LidarData__Sequence
{
  my_custom_msgs__msg__LidarData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} my_custom_msgs__msg__LidarData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MY_CUSTOM_MSGS__MSG__DETAIL__LIDAR_DATA__STRUCT_H_
