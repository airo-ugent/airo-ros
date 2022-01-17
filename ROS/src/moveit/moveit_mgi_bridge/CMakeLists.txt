cmake_minimum_required(VERSION 3.8)
project(moveit_mgi_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(mgi_bridge src/mgi_bridge.cpp)

ament_target_dependencies(mgi_bridge rclcpp std_msgs moveit_ros_planning_interface geometry_msgs)

target_include_directories(mgi_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(mgi_bridge PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS mgi_bridge
  DESTINATION lib/${PROJECT_NAME})


ament_package()
