cmake_minimum_required(VERSION 3.8)
project(planning_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Obstacle.msg"
  "msg/ObstacleArray.msg"
  "msg/ModeState.msg"
  "msg/ConeInfo.msg"
  "msg/ConeMapMsg.msg"
  DEPENDENCIES geometry_msgs std_msgs builtin_interfaces
)

ament_package()
