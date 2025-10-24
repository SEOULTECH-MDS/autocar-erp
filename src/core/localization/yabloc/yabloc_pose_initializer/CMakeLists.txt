cmake_minimum_required(VERSION 3.5)
project(yabloc_pose_initializer)

find_package(autoware_cmake REQUIRED)
autoware_package()

# ===================================================
# Eigen3
find_package(Eigen3 REQUIRED)

# PCL
find_package(PCL REQUIRED COMPONENTS common kdtree)

# Sophus
find_package(Sophus REQUIRED)

# OpenCV
find_package(OpenCV REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(python_cmake_module REQUIRED)
# Ensure legacy FindPythonInterp consumers see a valid interpreter
set(PYTHON_EXECUTABLE ${Python3_EXECUTABLE} CACHE FILEPATH "Python executable" FORCE)

# Interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/PoseWithCovarianceStamped.srv"
  DEPENDENCIES geometry_msgs
)

# ===================================================
# Executable
# Camera
set(YABLOC_POSE_INIT_LIB ${PROJECT_NAME}_lib)
ament_auto_add_library(${YABLOC_POSE_INIT_LIB}
  src/camera/lane_image.cpp
  src/camera/marker_module.cpp
  src/camera/projector_module.cpp
  src/camera/semantic_segmentation.cpp
  src/camera/camera_pose_initializer_core.cpp)
target_include_directories(${YABLOC_POSE_INIT_LIB} PUBLIC include)
target_include_directories(${YABLOC_POSE_INIT_LIB} SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${YABLOC_POSE_INIT_LIB} ${PCL_LIBRARIES} Sophus::Sophus)
ament_target_dependencies(${YABLOC_POSE_INIT_LIB} OpenCV)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${YABLOC_POSE_INIT_LIB} ${cpp_typesupport_target})
rosidl_target_interfaces(${YABLOC_POSE_INIT_LIB} ${PROJECT_NAME} "rosidl_typesupport_cpp")

rclcpp_components_register_node(${YABLOC_POSE_INIT_LIB}
  PLUGIN "yabloc::CameraPoseInitializer"
  EXECUTABLE ${PROJECT_NAME}_node
  EXECUTOR MultiThreadedExecutor
)

# ===================================================
ament_auto_package(INSTALL_TO_SHARE config launch)
