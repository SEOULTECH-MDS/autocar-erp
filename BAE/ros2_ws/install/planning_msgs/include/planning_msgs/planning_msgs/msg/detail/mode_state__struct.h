// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from planning_msgs:msg/ModeState.idl
// generated code does not contain a copyright notice

#ifndef PLANNING_MSGS__MSG__DETAIL__MODE_STATE__STRUCT_H_
#define PLANNING_MSGS__MSG__DETAIL__MODE_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'DRIVE'.
enum
{
  planning_msgs__msg__ModeState__DRIVE = 0
};

/// Constant 'PAUSE'.
enum
{
  planning_msgs__msg__ModeState__PAUSE = 1
};

/// Constant 'OBSTACLE_STATIC'.
enum
{
  planning_msgs__msg__ModeState__OBSTACLE_STATIC = 2
};

/// Constant 'OBSTACLE_DYNAMIC'.
enum
{
  planning_msgs__msg__ModeState__OBSTACLE_DYNAMIC = 3
};

/// Constant 'DELIVERY'.
enum
{
  planning_msgs__msg__ModeState__DELIVERY = 4
};

/// Constant 'PARKING'.
enum
{
  planning_msgs__msg__ModeState__PARKING = 5
};

/// Constant 'RETURN'.
enum
{
  planning_msgs__msg__ModeState__RETURN = 6
};

// Include directives for member types
// Member 'description'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/ModeState in the package planning_msgs.
typedef struct planning_msgs__msg__ModeState
{
  uint8_t current_mode;
  rosidl_runtime_c__String description;
} planning_msgs__msg__ModeState;

// Struct for a sequence of planning_msgs__msg__ModeState.
typedef struct planning_msgs__msg__ModeState__Sequence
{
  planning_msgs__msg__ModeState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} planning_msgs__msg__ModeState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PLANNING_MSGS__MSG__DETAIL__MODE_STATE__STRUCT_H_
