// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from my_custom_msgs:msg/LidarDatas.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "my_custom_msgs/msg/detail/lidar_datas__rosidl_typesupport_introspection_c.h"
#include "my_custom_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "my_custom_msgs/msg/detail/lidar_datas__functions.h"
#include "my_custom_msgs/msg/detail/lidar_datas__struct.h"


// Include directives for member types
// Member `data`
#include "my_custom_msgs/msg/lidar_data.h"
// Member `data`
#include "my_custom_msgs/msg/detail/lidar_data__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void LidarDatas__rosidl_typesupport_introspection_c__LidarDatas_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  my_custom_msgs__msg__LidarDatas__init(message_memory);
}

void LidarDatas__rosidl_typesupport_introspection_c__LidarDatas_fini_function(void * message_memory)
{
  my_custom_msgs__msg__LidarDatas__fini(message_memory);
}

size_t LidarDatas__rosidl_typesupport_introspection_c__size_function__LidarData__data(
  const void * untyped_member)
{
  const my_custom_msgs__msg__LidarData__Sequence * member =
    (const my_custom_msgs__msg__LidarData__Sequence *)(untyped_member);
  return member->size;
}

const void * LidarDatas__rosidl_typesupport_introspection_c__get_const_function__LidarData__data(
  const void * untyped_member, size_t index)
{
  const my_custom_msgs__msg__LidarData__Sequence * member =
    (const my_custom_msgs__msg__LidarData__Sequence *)(untyped_member);
  return &member->data[index];
}

void * LidarDatas__rosidl_typesupport_introspection_c__get_function__LidarData__data(
  void * untyped_member, size_t index)
{
  my_custom_msgs__msg__LidarData__Sequence * member =
    (my_custom_msgs__msg__LidarData__Sequence *)(untyped_member);
  return &member->data[index];
}

bool LidarDatas__rosidl_typesupport_introspection_c__resize_function__LidarData__data(
  void * untyped_member, size_t size)
{
  my_custom_msgs__msg__LidarData__Sequence * member =
    (my_custom_msgs__msg__LidarData__Sequence *)(untyped_member);
  my_custom_msgs__msg__LidarData__Sequence__fini(member);
  return my_custom_msgs__msg__LidarData__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember LidarDatas__rosidl_typesupport_introspection_c__LidarDatas_message_member_array[1] = {
  {
    "data",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(my_custom_msgs__msg__LidarDatas, data),  // bytes offset in struct
    NULL,  // default value
    LidarDatas__rosidl_typesupport_introspection_c__size_function__LidarData__data,  // size() function pointer
    LidarDatas__rosidl_typesupport_introspection_c__get_const_function__LidarData__data,  // get_const(index) function pointer
    LidarDatas__rosidl_typesupport_introspection_c__get_function__LidarData__data,  // get(index) function pointer
    LidarDatas__rosidl_typesupport_introspection_c__resize_function__LidarData__data  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers LidarDatas__rosidl_typesupport_introspection_c__LidarDatas_message_members = {
  "my_custom_msgs__msg",  // message namespace
  "LidarDatas",  // message name
  1,  // number of fields
  sizeof(my_custom_msgs__msg__LidarDatas),
  LidarDatas__rosidl_typesupport_introspection_c__LidarDatas_message_member_array,  // message members
  LidarDatas__rosidl_typesupport_introspection_c__LidarDatas_init_function,  // function to initialize message memory (memory has to be allocated)
  LidarDatas__rosidl_typesupport_introspection_c__LidarDatas_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t LidarDatas__rosidl_typesupport_introspection_c__LidarDatas_message_type_support_handle = {
  0,
  &LidarDatas__rosidl_typesupport_introspection_c__LidarDatas_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_my_custom_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, my_custom_msgs, msg, LidarDatas)() {
  LidarDatas__rosidl_typesupport_introspection_c__LidarDatas_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, my_custom_msgs, msg, LidarData)();
  if (!LidarDatas__rosidl_typesupport_introspection_c__LidarDatas_message_type_support_handle.typesupport_identifier) {
    LidarDatas__rosidl_typesupport_introspection_c__LidarDatas_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &LidarDatas__rosidl_typesupport_introspection_c__LidarDatas_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
