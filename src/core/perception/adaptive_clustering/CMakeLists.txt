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

# Create a shared library containing both components
add_library(${PROJECT_NAME}_lib SHARED
  src/adaptive_clustering.cpp
  src/adaptive_clustering_ground_removed.cpp
)
ament_target_dependencies(${PROJECT_NAME}_lib
  "rclcpp"
  "rclcpp_components"
  "std_msgs"
  "sensor_msgs"
  "visualization_msgs"
  "geometry_msgs"
  "pcl_conversions"
  "PCL"
)
target_link_libraries(${PROJECT_NAME}_lib
  ${PCL_LIBRARIES}
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

# Register the plugins from the library
rclcpp_components_register_node(
  ${PROJECT_NAME}_lib
  PLUGIN "perception::AdaptiveClusteringNode"
  EXECUTABLE adaptive_clustering_node
)
rclcpp_components_register_node(
  ${PROJECT_NAME}_lib
  PLUGIN "perception::AdaptiveClusteringGroundRemoved"
  EXECUTABLE adaptive_clustering_ground_removed_node
)


# --- Installation Rules ---
install(
  TARGETS ${PROJECT_NAME}_lib adaptive_clustering_node adaptive_clustering_ground_removed_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/
)

install(
    DIRECTORY msg
    DESTINATION share/${PROJECT_NAME}/
)

# Export dependencies and targets
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

ament_package()
