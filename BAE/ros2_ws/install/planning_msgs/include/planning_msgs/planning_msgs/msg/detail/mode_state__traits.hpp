// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from planning_msgs:msg/ModeState.idl
// generated code does not contain a copyright notice

#ifndef PLANNING_MSGS__MSG__DETAIL__MODE_STATE__TRAITS_HPP_
#define PLANNING_MSGS__MSG__DETAIL__MODE_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "planning_msgs/msg/detail/mode_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace planning_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const ModeState & msg,
  std::ostream & out)
{
  out << "{";
  // member: current_mode
  {
    out << "current_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.current_mode, out);
    out << ", ";
  }

  // member: description
  {
    out << "description: ";
    rosidl_generator_traits::value_to_yaml(msg.description, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ModeState & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: current_mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.current_mode, out);
    out << "\n";
  }

  // member: description
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "description: ";
    rosidl_generator_traits::value_to_yaml(msg.description, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ModeState & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace planning_msgs

namespace rosidl_generator_traits
{

[[deprecated("use planning_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const planning_msgs::msg::ModeState & msg,
  std::ostream & out, size_t indentation = 0)
{
  planning_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use planning_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const planning_msgs::msg::ModeState & msg)
{
  return planning_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<planning_msgs::msg::ModeState>()
{
  return "planning_msgs::msg::ModeState";
}

template<>
inline const char * name<planning_msgs::msg::ModeState>()
{
  return "planning_msgs/msg/ModeState";
}

template<>
struct has_fixed_size<planning_msgs::msg::ModeState>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<planning_msgs::msg::ModeState>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<planning_msgs::msg::ModeState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PLANNING_MSGS__MSG__DETAIL__MODE_STATE__TRAITS_HPP_
