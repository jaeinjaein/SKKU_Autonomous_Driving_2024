// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from my_custom_msgs:msg/LidarData.idl
// generated code does not contain a copyright notice

#ifndef MY_CUSTOM_MSGS__MSG__DETAIL__LIDAR_DATA__TRAITS_HPP_
#define MY_CUSTOM_MSGS__MSG__DETAIL__LIDAR_DATA__TRAITS_HPP_

#include "my_custom_msgs/msg/detail/lidar_data__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

namespace rosidl_generator_traits
{

inline void to_yaml(
  const my_custom_msgs::msg::LidarData & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: angle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "angle: ";
    value_to_yaml(msg.angle, out);
    out << "\n";
  }

  // member: distance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "distance: ";
    value_to_yaml(msg.distance, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const my_custom_msgs::msg::LidarData & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<my_custom_msgs::msg::LidarData>()
{
  return "my_custom_msgs::msg::LidarData";
}

template<>
inline const char * name<my_custom_msgs::msg::LidarData>()
{
  return "my_custom_msgs/msg/LidarData";
}

template<>
struct has_fixed_size<my_custom_msgs::msg::LidarData>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<my_custom_msgs::msg::LidarData>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<my_custom_msgs::msg::LidarData>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MY_CUSTOM_MSGS__MSG__DETAIL__LIDAR_DATA__TRAITS_HPP_
