cmake_minimum_required(VERSION 3.8)
project(erp42_visualizer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

# Install URDF and mesh files
install(DIRECTORY
  urdf
  meshes
  DESTINATION share/${PROJECT_NAME})
  
# Install RViz config files
install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME})

# Install python scripts as executable programs
install(PROGRAMS
  scripts/ackermann_to_joint_states.py
  DESTINATION lib/${PROJECT_NAME})

ament_package() 