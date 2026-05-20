# Copyright 2016-2025 Aeva, Inc. All rights reserved.
# 
# Please refer to the Aeva Software End-User License Agreement (EULA) associated with this source
# code for terms and conditions that govern your use of this software. Any use, reproduction,
# disclosure, or distribution of this software and related documentation outside the terms of the
# EULA is strictly prohibited. Please visit aeva.com/software-eula to review the EULA terms.

cmake_minimum_required(VERSION 3.10)

project(ros2_aeva_publisher)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(aeva_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

set(AEVA_API_ATLAS TRUE)
set(AEVA_API_BUILD_TYPE "ros2")
set(AEVA_API_MIN_VERSION "3.1.0")

find_package(AevaAPI CONFIG PATHS "/opt/aeva/atlas-api/cmake" "/opt/aeva/atlas-api-cuda/cmake")

include_directories(${AevaAPI_INCLUDE_DIRS}
	/opt/aeva/atlas-api/ros2/include
	)
link_directories(${AevaAPI_LIBRARY_DIRS})

set(DEPENDENCIES
  aeva_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2_msgs
  visualization_msgs
)


add_executable(ros2_aeva_publisher src/ros2_aeva_publisher.cc)
ament_target_dependencies(ros2_aeva_publisher ${DEPENDENCIES})

target_link_libraries(ros2_aeva_publisher
  ${rclcpp_LIBRARIES}
  AevaAPI
  AevaAPICallbacks
  AevaPublisherNode
)

add_library(AevaPublisherNode STATIC src/AevaPublisherNode.cc src/AevaPublisherNode.h)
target_include_directories(AevaPublisherNode PUBLIC /opt/aeva/atlas-api/ros2/include)
ament_target_dependencies(AevaPublisherNode ${DEPENDENCIES})
target_link_libraries(AevaPublisherNode AevaAPI)

add_library(AevaAPICallbacks STATIC src/AevaAPICallbacks.cc src/AevaAPICallbacks.h)
target_include_directories(AevaAPICallbacks PUBLIC /opt/aeva/atlas-api/ros2/include)
ament_target_dependencies(AevaAPICallbacks ${DEPENDENCIES})
target_link_libraries(AevaAPICallbacks AevaAPI)

install(TARGETS
  ros2_aeva_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
