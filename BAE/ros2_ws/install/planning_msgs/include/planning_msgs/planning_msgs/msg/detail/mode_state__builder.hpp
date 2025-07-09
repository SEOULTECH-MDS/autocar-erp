// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from planning_msgs:msg/ModeState.idl
// generated code does not contain a copyright notice

#ifndef PLANNING_MSGS__MSG__DETAIL__MODE_STATE__BUILDER_HPP_
#define PLANNING_MSGS__MSG__DETAIL__MODE_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "planning_msgs/msg/detail/mode_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace planning_msgs
{

namespace msg
{

namespace builder
{

class Init_ModeState_description
{
public:
  explicit Init_ModeState_description(::planning_msgs::msg::ModeState & msg)
  : msg_(msg)
  {}
  ::planning_msgs::msg::ModeState description(::planning_msgs::msg::ModeState::_description_type arg)
  {
    msg_.description = std::move(arg);
    return std::move(msg_);
  }

private:
  ::planning_msgs::msg::ModeState msg_;
};

class Init_ModeState_current_mode
{
public:
  Init_ModeState_current_mode()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ModeState_description current_mode(::planning_msgs::msg::ModeState::_current_mode_type arg)
  {
    msg_.current_mode = std::move(arg);
    return Init_ModeState_description(msg_);
  }

private:
  ::planning_msgs::msg::ModeState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::planning_msgs::msg::ModeState>()
{
  return planning_msgs::msg::builder::Init_ModeState_current_mode();
}

}  // namespace planning_msgs

#endif  // PLANNING_MSGS__MSG__DETAIL__MODE_STATE__BUILDER_HPP_
