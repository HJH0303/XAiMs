###############################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(xaims_localization)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)


add_executable(rviz_click_to_2d src/rviz_click_to_2d.cpp)
ament_target_dependencies(rviz_click_to_2d rclcpp geometry_msgs tf2 tf2_geometry_msgs)
install(TARGETS rviz_click_to_2d DESTINATION lib/${PROJECT_NAME})

add_executable(ekf_odom_pub src/ekf_odom_pub.cpp)
ament_target_dependencies(ekf_odom_pub
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
)
install(TARGETS
  ekf_odom_pub
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(odom_data_saver src/odom_data_saver.cpp)
ament_target_dependencies(odom_data_saver rclcpp nav_msgs)
install(TARGETS odom_data_saver DESTINATION lib/${PROJECT_NAME})

add_executable(pose_estimate_publisher src/pose_estimate_publisher.cpp)
ament_target_dependencies(pose_estimate_publisher rclcpp geometry_msgs tf2 tf2_geometry_msgs)
install(TARGETS pose_estimate_publisher DESTINATION lib/${PROJECT_NAME})


################################################################################
# Install
################################################################################
install(
  DIRECTORY launch rviz params
  DESTINATION share/${PROJECT_NAME}
)

################################################################################
# Macro for ament package
################################################################################
ament_package()