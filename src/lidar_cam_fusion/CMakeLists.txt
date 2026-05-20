cmake_minimum_required(VERSION 3.8)
project(lidar_cam_fusion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(message_filters REQUIRED)


add_executable(lidar_camera_overlay_node src/lidar_camera_overlay_node.cpp)
ament_target_dependencies(lidar_camera_overlay_node
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
  Eigen3
  message_filters
)
target_include_directories(lidar_camera_overlay_node PUBLIC
  ${EIGEN3_INCLUDE_DIR}
)

install(TARGETS
  lidar_camera_overlay_node
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(FILES
  overlay_config.yaml
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
