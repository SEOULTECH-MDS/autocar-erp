// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from planning_msgs:msg/ModeState.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "planning_msgs/msg/detail/mode_state__rosidl_typesupport_introspection_c.h"
#include "planning_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "planning_msgs/msg/detail/mode_state__functions.h"
#include "planning_msgs/msg/detail/mode_state__struct.h"


// Include directives for member types
// Member `description`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void planning_msgs__msg__ModeState__rosidl_typesupport_introspection_c__ModeState_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  planning_msgs__msg__ModeState__init(message_memory);
}

void planning_msgs__msg__ModeState__rosidl_typesupport_introspection_c__ModeState_fini_function(void * message_memory)
{
  planning_msgs__msg__ModeState__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember planning_msgs__msg__ModeState__rosidl_typesupport_introspection_c__ModeState_message_member_array[2] = {
  {
    "current_mode",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(planning_msgs__msg__ModeState, current_mode),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "description",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(planning_msgs__msg__ModeState, description),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers planning_msgs__msg__ModeState__rosidl_typesupport_introspection_c__ModeState_message_members = {
  "planning_msgs__msg",  // message namespace
  "ModeState",  // message name
  2,  // number of fields
  sizeof(planning_msgs__msg__ModeState),
  planning_msgs__msg__ModeState__rosidl_typesupport_introspection_c__ModeState_message_member_array,  // message members
  planning_msgs__msg__ModeState__rosidl_typesupport_introspection_c__ModeState_init_function,  // function to initialize message memory (memory has to be allocated)
  planning_msgs__msg__ModeState__rosidl_typesupport_introspection_c__ModeState_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t planning_msgs__msg__ModeState__rosidl_typesupport_introspection_c__ModeState_message_type_support_handle = {
  0,
  &planning_msgs__msg__ModeState__rosidl_typesupport_introspection_c__ModeState_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_planning_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, planning_msgs, msg, ModeState)() {
  if (!planning_msgs__msg__ModeState__rosidl_typesupport_introspection_c__ModeState_message_type_support_handle.typesupport_identifier) {
    planning_msgs__msg__ModeState__rosidl_typesupport_introspection_c__ModeState_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &planning_msgs__msg__ModeState__rosidl_typesupport_introspection_c__ModeState_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
