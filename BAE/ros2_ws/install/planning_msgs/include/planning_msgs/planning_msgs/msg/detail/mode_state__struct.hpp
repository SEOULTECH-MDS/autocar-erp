// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from planning_msgs:msg/ModeState.idl
// generated code does not contain a copyright notice

#ifndef PLANNING_MSGS__MSG__DETAIL__MODE_STATE__STRUCT_HPP_
#define PLANNING_MSGS__MSG__DETAIL__MODE_STATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__planning_msgs__msg__ModeState __attribute__((deprecated))
#else
# define DEPRECATED__planning_msgs__msg__ModeState __declspec(deprecated)
#endif

namespace planning_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ModeState_
{
  using Type = ModeState_<ContainerAllocator>;

  explicit ModeState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->current_mode = 0;
      this->description = "";
    }
  }

  explicit ModeState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : description(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->current_mode = 0;
      this->description = "";
    }
  }

  // field types and members
  using _current_mode_type =
    uint8_t;
  _current_mode_type current_mode;
  using _description_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _description_type description;

  // setters for named parameter idiom
  Type & set__current_mode(
    const uint8_t & _arg)
  {
    this->current_mode = _arg;
    return *this;
  }
  Type & set__description(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->description = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t DRIVE =
    0u;
  static constexpr uint8_t PAUSE =
    1u;
  static constexpr uint8_t OBSTACLE_STATIC =
    2u;
  static constexpr uint8_t OBSTACLE_DYNAMIC =
    3u;
  static constexpr uint8_t DELIVERY =
    4u;
  static constexpr uint8_t PARKING =
    5u;
  static constexpr uint8_t RETURN =
    6u;

  // pointer types
  using RawPtr =
    planning_msgs::msg::ModeState_<ContainerAllocator> *;
  using ConstRawPtr =
    const planning_msgs::msg::ModeState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<planning_msgs::msg::ModeState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<planning_msgs::msg::ModeState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      planning_msgs::msg::ModeState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<planning_msgs::msg::ModeState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      planning_msgs::msg::ModeState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<planning_msgs::msg::ModeState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<planning_msgs::msg::ModeState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<planning_msgs::msg::ModeState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__planning_msgs__msg__ModeState
    std::shared_ptr<planning_msgs::msg::ModeState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__planning_msgs__msg__ModeState
    std::shared_ptr<planning_msgs::msg::ModeState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ModeState_ & other) const
  {
    if (this->current_mode != other.current_mode) {
      return false;
    }
    if (this->description != other.description) {
      return false;
    }
    return true;
  }
  bool operator!=(const ModeState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ModeState_

// alias to use template instance with default allocator
using ModeState =
  planning_msgs::msg::ModeState_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ModeState_<ContainerAllocator>::DRIVE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ModeState_<ContainerAllocator>::PAUSE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ModeState_<ContainerAllocator>::OBSTACLE_STATIC;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ModeState_<ContainerAllocator>::OBSTACLE_DYNAMIC;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ModeState_<ContainerAllocator>::DELIVERY;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ModeState_<ContainerAllocator>::PARKING;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ModeState_<ContainerAllocator>::RETURN;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace planning_msgs

#endif  // PLANNING_MSGS__MSG__DETAIL__MODE_STATE__STRUCT_HPP_
