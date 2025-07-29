cmake_minimum_required(VERSION 3.14)
project(autoware_component_interface_specs)

find_package(autoware_cmake REQUIRED)
autoware_package()

if(BUILD_TESTING)
  find_package(autoware_control_msgs REQUIRED)
  find_package(autoware_perception_msgs REQUIRED)
  find_package(autoware_vehicle_msgs REQUIRED)

  ament_auto_add_gtest(gtest_${PROJECT_NAME}
    test/gtest_main.cpp
    test/test_planning.cpp
    test/test_control.cpp
    test/test_localization.cpp
    test/test_map.cpp
    test/test_perception.cpp
    test/test_vehicle.cpp
  )
  ament_target_dependencies(gtest_${PROJECT_NAME}
    autoware_control_msgs
    autoware_perception_msgs
    autoware_vehicle_msgs
  )
endif()

ament_auto_package()
