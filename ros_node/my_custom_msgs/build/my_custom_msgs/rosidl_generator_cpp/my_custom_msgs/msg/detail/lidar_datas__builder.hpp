// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from my_custom_msgs:msg/LidarDatas.idl
// generated code does not contain a copyright notice

#ifndef MY_CUSTOM_MSGS__MSG__DETAIL__LIDAR_DATAS__BUILDER_HPP_
#define MY_CUSTOM_MSGS__MSG__DETAIL__LIDAR_DATAS__BUILDER_HPP_

#include "my_custom_msgs/msg/detail/lidar_datas__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace my_custom_msgs
{

namespace msg
{

namespace builder
{

class Init_LidarDatas_data
{
public:
  Init_LidarDatas_data()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::my_custom_msgs::msg::LidarDatas data(::my_custom_msgs::msg::LidarDatas::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::my_custom_msgs::msg::LidarDatas msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::my_custom_msgs::msg::LidarDatas>()
{
  return my_custom_msgs::msg::builder::Init_LidarDatas_data();
}

}  // namespace my_custom_msgs

#endif  // MY_CUSTOM_MSGS__MSG__DETAIL__LIDAR_DATAS__BUILDER_HPP_
