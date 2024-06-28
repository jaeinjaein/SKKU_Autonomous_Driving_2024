// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from my_custom_msgs:msg/LidarData.idl
// generated code does not contain a copyright notice

#ifndef MY_CUSTOM_MSGS__MSG__DETAIL__LIDAR_DATA__BUILDER_HPP_
#define MY_CUSTOM_MSGS__MSG__DETAIL__LIDAR_DATA__BUILDER_HPP_

#include "my_custom_msgs/msg/detail/lidar_data__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace my_custom_msgs
{

namespace msg
{

namespace builder
{

class Init_LidarData_distance
{
public:
  explicit Init_LidarData_distance(::my_custom_msgs::msg::LidarData & msg)
  : msg_(msg)
  {}
  ::my_custom_msgs::msg::LidarData distance(::my_custom_msgs::msg::LidarData::_distance_type arg)
  {
    msg_.distance = std::move(arg);
    return std::move(msg_);
  }

private:
  ::my_custom_msgs::msg::LidarData msg_;
};

class Init_LidarData_angle
{
public:
  Init_LidarData_angle()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LidarData_distance angle(::my_custom_msgs::msg::LidarData::_angle_type arg)
  {
    msg_.angle = std::move(arg);
    return Init_LidarData_distance(msg_);
  }

private:
  ::my_custom_msgs::msg::LidarData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::my_custom_msgs::msg::LidarData>()
{
  return my_custom_msgs::msg::builder::Init_LidarData_angle();
}

}  // namespace my_custom_msgs

#endif  // MY_CUSTOM_MSGS__MSG__DETAIL__LIDAR_DATA__BUILDER_HPP_
