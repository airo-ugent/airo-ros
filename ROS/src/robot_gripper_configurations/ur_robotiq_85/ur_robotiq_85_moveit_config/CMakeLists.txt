cmake_minimum_required(VERSION 3.8)
project(ur_robotiq_85_moveit_config)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    config
    srdf
    rviz
  DESTINATION
    share/${PROJECT_NAME}
)

ament_package()
