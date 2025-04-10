cmake_minimum_required(VERSION 3.5)
project(autocar_nav)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)


ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY
  config
  data
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  nodes/
  DESTINATION lib/${PROJECT_NAME}
  FILES_MATCHING
  PATTERN "*.py"
  PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
              GROUP_EXECUTE GROUP_READ
              WORLD_EXECUTE WORLD_READ
)

install(PROGRAMS
  nodes/autocar_tf_ros2.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME autocar_tf_ros2
)

install(PROGRAMS
  nodes/hitech_test_pub.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME hitech_test_pub
)

install(PROGRAMS
  nodes/mpc_tracker.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME mpc_tracker
)

install(PROGRAMS
  nodes/global_planner.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME global_planner
)

install(PROGRAMS
  nodes/path_selector.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME path_selector
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
