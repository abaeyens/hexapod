cmake_minimum_required(VERSION 3.5)
project(robot_description)

find_package(ament_cmake REQUIRED)

install(
  FILES
    robot_description.launch.py
    hexapod.urdf.xacro
    properties.urdf.xacro
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
