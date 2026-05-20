# Copyright 2016-2025 Aeva, Inc. All rights reserved.
# 
# Please refer to the Aeva Software End-User License Agreement (EULA) associated with this source
# code for terms and conditions that govern your use of this software. Any use, reproduction,
# disclosure, or distribution of this software and related documentation outside the terms of the
# EULA is strictly prohibited. Please visit aeva.com/software-eula to review the EULA terms.

cmake_minimum_required(VERSION 3.10)
project(aeva_msgs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Detection3D.msg"
  "msg/Detection3DArray.msg"
  "msg/EstimatedDetectionRangeBin.msg"
  "msg/EstimatedDetectionRange.msg"
  "msg/Health.msg"
  "msg/PointCloudMetadata.msg"
  "msg/RawData.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs vision_msgs
 )

ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)

ament_package()
