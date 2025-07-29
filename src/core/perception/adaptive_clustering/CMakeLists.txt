cmake_minimum_required(VERSION 3.8)
project(adaptive_clustering)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate C++ headers from msg files
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ClusterArray.msg"
  DEPENDENCIES std_msgs sensor_msgs
)

# Create a library from the source file
add_library(${PROJECT_NAME}_lib src/adaptive_clustering.cpp)
target_include_directories(${PROJECT_NAME}_lib
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:.>
)
# Link against ROS dependencies using the ament macro
ament_target_dependencies(${PROJECT_NAME}_lib
  "rclcpp"
  "rclcpp_components"
  "std_msgs"
  "sensor_msgs"
  "visualization_msgs"
  "geometry_msgs"
  "pcl_conversions"
)
# Link against non-ament and generated message libraries directly
target_link_libraries(${PROJECT_NAME}_lib
  ${PCL_LIBRARIES}
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

# Register the library as a composable node component
rclcpp_components_register_node(
  ${PROJECT_NAME}_lib PLUGIN "perception::AdaptiveClusteringNode" EXECUTABLE ${PROJECT_NAME}_node
)

# --- Installation Rules ---
install(
  TARGETS ${PROJECT_NAME}_lib ${PROJECT_NAME}_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/
)

# Export dependencies and targets for downstream packages
ament_export_dependencies(
  "rclcpp"
  "rclcpp_components"
  "std_msgs"
  "sensor_msgs"
  "visualization_msgs"
  "geometry_msgs"
  "pcl_conversions"
  "rosidl_default_runtime"
)
ament_export_libraries(${PROJECT_NAME}_lib)

ament_package()
