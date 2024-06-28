// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from my_custom_msgs:msg/LidarDatas.idl
// generated code does not contain a copyright notice

#ifndef MY_CUSTOM_MSGS__MSG__DETAIL__LIDAR_DATAS__STRUCT_HPP_
#define MY_CUSTOM_MSGS__MSG__DETAIL__LIDAR_DATAS__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'data'
#include "my_custom_msgs/msg/detail/lidar_data__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__my_custom_msgs__msg__LidarDatas __attribute__((deprecated))
#else
# define DEPRECATED__my_custom_msgs__msg__LidarDatas __declspec(deprecated)
#endif

namespace my_custom_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct LidarDatas_
{
  using Type = LidarDatas_<ContainerAllocator>;

  explicit LidarDatas_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit LidarDatas_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _data_type =
    std::vector<my_custom_msgs::msg::LidarData_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<my_custom_msgs::msg::LidarData_<ContainerAllocator>>>;
  _data_type data;

  // setters for named parameter idiom
  Type & set__data(
    const std::vector<my_custom_msgs::msg::LidarData_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<my_custom_msgs::msg::LidarData_<ContainerAllocator>>> & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    my_custom_msgs::msg::LidarDatas_<ContainerAllocator> *;
  using ConstRawPtr =
    const my_custom_msgs::msg::LidarDatas_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<my_custom_msgs::msg::LidarDatas_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<my_custom_msgs::msg::LidarDatas_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      my_custom_msgs::msg::LidarDatas_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<my_custom_msgs::msg::LidarDatas_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      my_custom_msgs::msg::LidarDatas_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<my_custom_msgs::msg::LidarDatas_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<my_custom_msgs::msg::LidarDatas_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<my_custom_msgs::msg::LidarDatas_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__my_custom_msgs__msg__LidarDatas
    std::shared_ptr<my_custom_msgs::msg::LidarDatas_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__my_custom_msgs__msg__LidarDatas
    std::shared_ptr<my_custom_msgs::msg::LidarDatas_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LidarDatas_ & other) const
  {
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const LidarDatas_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LidarDatas_

// alias to use template instance with default allocator
using LidarDatas =
  my_custom_msgs::msg::LidarDatas_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace my_custom_msgs

#endif  // MY_CUSTOM_MSGS__MSG__DETAIL__LIDAR_DATAS__STRUCT_HPP_
