cmake_minimum_required(VERSION 3.8)
project(airo_moveit_utils)

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(tf2_ros)
find_package(rclcpp)

set(THIS_PACKAGE_DEPENDS
  tf2_ros
  rclcpp
)
# add executable for transform streamer
add_executable(transform_streamer src/transform_streamer.cpp)
ament_target_dependencies(transform_streamer ${THIS_PACKAGE_DEPENDS})

# install executables to make them callable with ros2 run
install(TARGETS transform_streamer
  DESTINATION lib/${PROJECT_NAME})

set(ament_cmake_copyright_FOUND TRUE)

ament_package()
