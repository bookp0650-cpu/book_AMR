cmake_minimum_required(VERSION 3.8)
project(my_scan_merger)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED) 

add_executable(scan_merger src/scan_merger.cpp)
ament_target_dependencies(scan_merger
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
target_link_libraries(scan_merger
  yaml-cpp
)

install(TARGETS scan_merger
  DESTINATION lib/${PROJECT_NAME})

ament_package()
