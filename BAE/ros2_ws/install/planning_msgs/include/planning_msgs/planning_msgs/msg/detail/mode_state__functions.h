// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from planning_msgs:msg/ModeState.idl
// generated code does not contain a copyright notice

#ifndef PLANNING_MSGS__MSG__DETAIL__MODE_STATE__FUNCTIONS_H_
#define PLANNING_MSGS__MSG__DETAIL__MODE_STATE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "planning_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "planning_msgs/msg/detail/mode_state__struct.h"

/// Initialize msg/ModeState message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * planning_msgs__msg__ModeState
 * )) before or use
 * planning_msgs__msg__ModeState__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_planning_msgs
bool
planning_msgs__msg__ModeState__init(planning_msgs__msg__ModeState * msg);

/// Finalize msg/ModeState message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_planning_msgs
void
planning_msgs__msg__ModeState__fini(planning_msgs__msg__ModeState * msg);

/// Create msg/ModeState message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * planning_msgs__msg__ModeState__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_planning_msgs
planning_msgs__msg__ModeState *
planning_msgs__msg__ModeState__create();

/// Destroy msg/ModeState message.
/**
 * It calls
 * planning_msgs__msg__ModeState__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_planning_msgs
void
planning_msgs__msg__ModeState__destroy(planning_msgs__msg__ModeState * msg);

/// Check for msg/ModeState message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_planning_msgs
bool
planning_msgs__msg__ModeState__are_equal(const planning_msgs__msg__ModeState * lhs, const planning_msgs__msg__ModeState * rhs);

/// Copy a msg/ModeState message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_planning_msgs
bool
planning_msgs__msg__ModeState__copy(
  const planning_msgs__msg__ModeState * input,
  planning_msgs__msg__ModeState * output);

/// Initialize array of msg/ModeState messages.
/**
 * It allocates the memory for the number of elements and calls
 * planning_msgs__msg__ModeState__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_planning_msgs
bool
planning_msgs__msg__ModeState__Sequence__init(planning_msgs__msg__ModeState__Sequence * array, size_t size);

/// Finalize array of msg/ModeState messages.
/**
 * It calls
 * planning_msgs__msg__ModeState__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_planning_msgs
void
planning_msgs__msg__ModeState__Sequence__fini(planning_msgs__msg__ModeState__Sequence * array);

/// Create array of msg/ModeState messages.
/**
 * It allocates the memory for the array and calls
 * planning_msgs__msg__ModeState__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_planning_msgs
planning_msgs__msg__ModeState__Sequence *
planning_msgs__msg__ModeState__Sequence__create(size_t size);

/// Destroy array of msg/ModeState messages.
/**
 * It calls
 * planning_msgs__msg__ModeState__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_planning_msgs
void
planning_msgs__msg__ModeState__Sequence__destroy(planning_msgs__msg__ModeState__Sequence * array);

/// Check for msg/ModeState message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_planning_msgs
bool
planning_msgs__msg__ModeState__Sequence__are_equal(const planning_msgs__msg__ModeState__Sequence * lhs, const planning_msgs__msg__ModeState__Sequence * rhs);

/// Copy an array of msg/ModeState messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_planning_msgs
bool
planning_msgs__msg__ModeState__Sequence__copy(
  const planning_msgs__msg__ModeState__Sequence * input,
  planning_msgs__msg__ModeState__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // PLANNING_MSGS__MSG__DETAIL__MODE_STATE__FUNCTIONS_H_
