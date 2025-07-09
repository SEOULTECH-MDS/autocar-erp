// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from planning_msgs:msg/ModeState.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "planning_msgs/msg/detail/mode_state__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace planning_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void ModeState_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) planning_msgs::msg::ModeState(_init);
}

void ModeState_fini_function(void * message_memory)
{
  auto typed_message = static_cast<planning_msgs::msg::ModeState *>(message_memory);
  typed_message->~ModeState();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember ModeState_message_member_array[2] = {
  {
    "current_mode",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(planning_msgs::msg::ModeState, current_mode),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "description",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(planning_msgs::msg::ModeState, description),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers ModeState_message_members = {
  "planning_msgs::msg",  // message namespace
  "ModeState",  // message name
  2,  // number of fields
  sizeof(planning_msgs::msg::ModeState),
  ModeState_message_member_array,  // message members
  ModeState_init_function,  // function to initialize message memory (memory has to be allocated)
  ModeState_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t ModeState_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &ModeState_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace planning_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<planning_msgs::msg::ModeState>()
{
  return &::planning_msgs::msg::rosidl_typesupport_introspection_cpp::ModeState_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, planning_msgs, msg, ModeState)() {
  return &::planning_msgs::msg::rosidl_typesupport_introspection_cpp::ModeState_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
