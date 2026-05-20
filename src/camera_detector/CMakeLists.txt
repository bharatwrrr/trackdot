cmake_minimum_required(VERSION 3.8)
project(camera_detector)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# -------------------------------
# Find required packages
# -------------------------------
find_package(ament_cmake REQUIRED)

# ROS 2
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(trackdot_msgs REQUIRED)

# -------------------------------
# GStreamer (system libs)
# -------------------------------
find_package(PkgConfig REQUIRED)
find_package(CUDAToolkit REQUIRED)

pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
pkg_check_modules(GST_APP REQUIRED gstreamer-app-1.0)

# -------------------------------
# Include directories
# -------------------------------
include_directories(
  include
  ${GSTREAMER_INCLUDE_DIRS}
  ${GST_APP_INCLUDE_DIRS}
)

link_directories(
  ${GSTREAMER_LIBRARY_DIRS}
  ${GST_APP_LIBRARY_DIRS}
  /opt/nvidia/deepstream/deepstream/lib
)

# -------------------------------
# Executable
# -------------------------------
add_executable(camera_detector_node
  src/camera_detector_node.cpp   # <-- adjust if different
)

# -------------------------------
# ROS dependencies
# -------------------------------
ament_target_dependencies(camera_detector_node
  rclcpp
  sensor_msgs
  vision_msgs
  trackdot_msgs
)

# -------------------------------
# Link system libraries
# -------------------------------
target_link_libraries(camera_detector_node
  ${GSTREAMER_LIBRARIES}
  ${GST_APP_LIBRARIES}
)

# -------------------------------
# DeepStream (IMPORTANT)
# -------------------------------
# These headers:
#   gstnvdsmeta.h
#   nvds_yml_parser.h
# come from DeepStream

include_directories(
  /opt/nvidia/deepstream/deepstream/sources/includes
)

target_link_libraries(camera_detector_node
  nvdsgst_meta
  nvds_meta
  nvds_utils
  nvds_yml_parser
)

# -------------------------------
# Install executable
# -------------------------------
install(TARGETS
  camera_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

# -------------------------------
# Install launch + config
# -------------------------------
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  scripts/viz_overlay.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config/
)

# -------------------------------
# DeepStream custom parser library (.so)
# -------------------------------
# -------------------------------
# PGIE custom parser
# -------------------------------
add_library(nvdsinfer_custom_impl SHARED
  src/nvdsinfer_custom_impl.cpp
)

target_include_directories(nvdsinfer_custom_impl PRIVATE
  include
  /opt/nvidia/deepstream/deepstream/sources/includes
)

target_link_libraries(nvdsinfer_custom_impl
  nvinfer
  nvds_inferutils
  CUDA::cudart
)
set_target_properties(nvdsinfer_custom_impl PROPERTIES PREFIX "")

install(TARGETS nvdsinfer_custom_impl
  DESTINATION lib
)

# -------------------------------
# SGIE custom parser
# -------------------------------
add_library(nvdsinfer_custom_impl_sgie SHARED
  src/nvdsinfer_custom_impl_sgie.cpp
)

target_include_directories(nvdsinfer_custom_impl_sgie PRIVATE
  include
  /opt/nvidia/deepstream/deepstream/sources/includes
)

target_link_libraries(nvdsinfer_custom_impl_sgie
  nvinfer
  nvds_inferutils
  CUDA::cudart
)
set_target_properties(nvdsinfer_custom_impl_sgie PROPERTIES PREFIX "")

install(TARGETS nvdsinfer_custom_impl_sgie
  DESTINATION lib
)
# -------------------------------
# Ament package
# -------------------------------
ament_package()
