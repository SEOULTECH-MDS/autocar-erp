cmake_minimum_required(VERSION 3.14)
project(autoware_map_loader)

find_package(autoware_cmake REQUIRED)
autoware_package()


find_package(autoware_component_interface_specs REQUIRED)
find_package(autoware_geography_utils REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)

ament_auto_add_library(pointcloud_map_loader_node SHARED
  src/pointcloud_map_loader/pointcloud_map_loader_node.cpp
  src/pointcloud_map_loader/pointcloud_map_loader_module.cpp
  src/pointcloud_map_loader/partial_map_loader_module.cpp
  src/pointcloud_map_loader/differential_map_loader_module.cpp
  src/pointcloud_map_loader/selected_map_loader_module.cpp
  src/pointcloud_map_loader/utils.cpp
)
target_link_libraries(pointcloud_map_loader_node ${PCL_LIBRARIES})
target_link_libraries(pointcloud_map_loader_node yaml-cpp)

target_include_directories(pointcloud_map_loader_node
  SYSTEM PUBLIC
  ${PCL_INCLUDE_DIRS}
)

rclcpp_components_register_node(pointcloud_map_loader_node
  PLUGIN "autoware::map_loader::PointCloudMapLoaderNode"
  EXECUTABLE autoware_pointcloud_map_loader
)

ament_auto_add_library(lanelet2_map_loader_node SHARED
  src/lanelet2_map_loader/lanelet2_map_loader_node.cpp
)
ament_target_dependencies(
  lanelet2_map_loader_node
  autoware_component_interface_specs
  autoware_geography_utils
)

target_include_directories(lanelet2_map_loader_node
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

rclcpp_components_register_node(lanelet2_map_loader_node
  PLUGIN "autoware::map_loader::Lanelet2MapLoaderNode"
  EXECUTABLE autoware_lanelet2_map_loader
)


install(PROGRAMS
  script/map_hash_generator
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
