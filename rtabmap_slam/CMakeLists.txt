cmake_minimum_required(VERSION 3.5)
project(rtabmap_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# To suppress PCL_ROOT warning
if(POLICY CMP0074)
    cmake_policy(SET CMP0074 NEW)
endif()

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rtabmap_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS date_time)        # rtabmap_util 관련 빌드에러 해결용
find_package(rtabmap_util REQUIRED)
find_package(rtabmap_sync REQUIRED)
# alice4 커스터마이즈: CoreWrapper.h 가 alice_localization_msgs / aeirobot_toolbox 의
# QoS 프로파일을 사용한다. tf2_geometry_msgs 는 좌표 변환 헬퍼.
# alice_localization_msgs(로보컵 pelvis pose override)는 선택 — 있으면 HAVE_ALICE_LOCALIZATION_MSGS 로 켠다.
# 실기 축소 워크스페이스(Orin)엔 없고 package.xml 은 AEIROBOT_ROBOCUP=1 일 때만 의존을 선언한다 (2026-09-08).
find_package(alice_localization_msgs QUIET)
if(alice_localization_msgs_FOUND)
  add_compile_definitions(HAVE_ALICE_LOCALIZATION_MSGS)
  set(ALICE_LOC_MSGS_DEP alice_localization_msgs)
  message(STATUS "rtabmap_slam: alice_localization_msgs 발견 — robotPoseInfo override 활성")
else()
  set(ALICE_LOC_MSGS_DEP "")
  message(STATUS "rtabmap_slam: alice_localization_msgs 없음 — robotPoseInfo override 비활성")
endif()
find_package(tf2_geometry_msgs REQUIRED)
find_package(aeirobot_toolbox REQUIRED)

#optional
find_package(apriltag_msgs)
find_package(nav2_msgs)

IF(WIN32)
add_compile_options(-bigobj)
ENDIF(WIN32)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# libraries
SET(Libraries
   Boost                                                 # rtabmap_util 관련 빌드에러 해결용
   cv_bridge
   geometry_msgs
   nav_msgs
   rclcpp
   rclcpp_components
   sensor_msgs
   std_msgs
   std_srvs
   tf2
   tf2_ros
   visualization_msgs
   rtabmap_msgs
   rtabmap_util
   rtabmap_sync
   tf2_geometry_msgs                # alice4: 좌표 변환 헬퍼
   ${ALICE_LOC_MSGS_DEP}            # alice4: 커스텀 메시지 타입 (선택)
   aeirobot_toolbox                 # alice4: QoS 프로파일 (qos_profiles.hpp)
)

if("$ENV{ROS_DISTRO}" STRLESS "jazzy")
  add_definitions(-DPRE_ROS_JAZZY)
endif()

###########
## Build ##
###########

SET(rtabmap_slam_plugins_lib_src
   src/CoreWrapper.cpp
)

# If apriltag_msgs is found, add definition
IF(apriltag_msgs_FOUND)
MESSAGE(STATUS "WITH apriltag_msgs")
ADD_DEFINITIONS("-DWITH_APRILTAG_MSGS")
SET(Libraries
   ${Libraries}
   apriltag_msgs
)
ENDIF(apriltag_msgs_FOUND)

# If nav2_msgs is found, add definition
IF(nav2_msgs_FOUND)
MESSAGE(STATUS "WITH nav2_msgs")
ADD_DEFINITIONS("-DWITH_NAV2_MSGS")
SET(Libraries
   ${Libraries}
   nav2_msgs
)
IF(${nav2_msgs_VERSION_MAJOR} EQUAL 0)
   ADD_DEFINITIONS("-DNAV_MSGS_FOXY")
ENDIF()
ENDIF(nav2_msgs_FOUND)

############################
## Declare a cpp library
############################
add_library(rtabmap_slam_plugins SHARED
   ${rtabmap_slam_plugins_lib_src}
)
target_include_directories(rtabmap_slam_plugins
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(rtabmap_slam_plugins ${Libraries})

rclcpp_components_register_nodes(rtabmap_slam_plugins "rtabmap_slam::CoreWrapper")

add_executable(rtabmap_node src/CoreNode.cpp)
ament_target_dependencies(rtabmap_node ${Libraries})
target_link_libraries(rtabmap_node rtabmap_slam_plugins)
set_target_properties(rtabmap_node PROPERTIES OUTPUT_NAME "rtabmap")

#############
## Install ##
#############

ament_export_dependencies(${Libraries})
ament_export_include_directories(include)
ament_export_targets(${PROJECT_NAME})          # To include downstream with targets
ament_export_libraries(rtabmap_slam_plugins) # To include downstream without targets

install(TARGETS 
   rtabmap_slam_plugins
   EXPORT ${PROJECT_NAME}
   ARCHIVE DESTINATION lib
   LIBRARY DESTINATION lib
   RUNTIME DESTINATION bin
)

install(TARGETS 
   rtabmap_node 
   DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
   DESTINATION include
   FILES_MATCHING PATTERN "*.h"
)

ament_package()
