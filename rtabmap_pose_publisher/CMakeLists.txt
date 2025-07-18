cmake_minimum_required(VERSION 3.8)
project(rtabmap_pose_publisher)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp            REQUIRED)
find_package(rtabmap_msgs      REQUIRED)
find_package(nav_msgs          REQUIRED)
find_package(geometry_msgs     REQUIRED)
find_package(tf2_ros           REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(aeirobot_toolbox REQUIRED)
find_package(tf2 REQUIRED)
find_package(alice4_localization_msgs REQUIRED)

add_executable(rtabmap_pose_publisher
  src/rtabmap_pose_publisher.cpp
)

ament_target_dependencies(rtabmap_pose_publisher
  rclcpp
  rtabmap_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  aeirobot_toolbox
  tf2
  alice4_localization_msgs
)

install(TARGETS rtabmap_pose_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
