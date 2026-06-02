cmake_minimum_required(VERSION 3.8)
project(gazebo_env_setup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(pose_tf_broadcaster src/pose_tf_broadcaster.cpp)

ament_target_dependencies(pose_tf_broadcaster
  rclcpp
  tf2_ros
  tf2_msgs
  geometry_msgs
)

install(TARGETS
  pose_tf_broadcaster
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.py"
)

ament_package()
