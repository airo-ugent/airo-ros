cmake_minimum_required(VERSION 3.8)
project(airo_moveit_mgi_bridge)


# mgi bridge executable name
set (MGI_EXECUTABLE moveit_mgi_bridge)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(airo_moveit_mgi_bridge_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

set(PACKAGE_DEPENDENCIES
rclcpp
geometry_msgs
airo_moveit_mgi_bridge_msgs
moveit_ros_planning_interface
)

# executable for the bridge
add_executable(${MGI_EXECUTABLE} src/airo_moveit_mgi_bridge.cpp)
ament_target_dependencies(${MGI_EXECUTABLE} ${PACKAGE_DEPENDENCIES})

# make header files available
target_include_directories(${MGI_EXECUTABLE} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# install
install(TARGETS ${MGI_EXECUTABLE}
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
