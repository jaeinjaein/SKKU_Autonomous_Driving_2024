// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from my_custom_msgs:msg/LidarDatas.idl
// generated code does not contain a copyright notice

#ifndef MY_CUSTOM_MSGS__MSG__DETAIL__LIDAR_DATAS__TRAITS_HPP_
#define MY_CUSTOM_MSGS__MSG__DETAIL__LIDAR_DATAS__TRAITS_HPP_

#include "my_custom_msgs/msg/detail/lidar_datas__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

// Include directives for member types
// Member 'data'
#include "my_custom_msgs/msg/detail/lidar_data__traits.hpp"

namespace rosidl_generator_traits
{

inline void to_yaml(
  const my_custom_msgs::msg::LidarDatas & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.data.size() == 0) {
      out << "data: []\n";
    } else {
      out << "data:\n";
      for (auto item : msg.data) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const my_custom_msgs::msg::LidarDatas & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<my_custom_msgs::msg::LidarDatas>()
{
  return "my_custom_msgs::msg::LidarDatas";
}

template<>
inline const char * name<my_custom_msgs::msg::LidarDatas>()
{
  return "my_custom_msgs/msg/LidarDatas";
}

template<>
struct has_fixed_size<my_custom_msgs::msg::LidarDatas>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<my_custom_msgs::msg::LidarDatas>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<my_custom_msgs::msg::LidarDatas>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MY_CUSTOM_MSGS__MSG__DETAIL__LIDAR_DATAS__TRAITS_HPP_
