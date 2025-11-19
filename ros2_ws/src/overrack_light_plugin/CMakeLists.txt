cmake_minimum_required(VERSION 3.10)
project(overrack_light_plugin)

# ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Gazebo Classic
find_package(gazebo REQUIRED)

# Ensure include + link directories are set correctly
include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${gazebo_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${gazebo_LIBRARY_DIRS}
)

add_library(torch_light_plugin SHARED src/torch_light_plugin.cpp)
target_compile_features(torch_light_plugin PUBLIC cxx_std_17)

ament_target_dependencies(torch_light_plugin
  rclcpp
  std_msgs
)

target_link_libraries(torch_light_plugin
  ${GAZEBO_LIBRARIES}
  gazebo
  gazebo_rendering
  gazebo_physics
  gazebo_common
)

install(
  TARGETS torch_light_plugin
  LIBRARY DESTINATION lib
)

ament_package()
