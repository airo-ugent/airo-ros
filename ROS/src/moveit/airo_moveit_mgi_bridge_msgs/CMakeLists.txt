
cmake_minimum_required(VERSION 3.8)
project(airo_moveit_mgi_bridge_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(msg_files
  #"msg/GripperCmd.msg"
  "srv/MoveToPose.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

ament_package()
