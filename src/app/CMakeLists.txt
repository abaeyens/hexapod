cmake_minimum_required(VERSION 3.8)
project(app)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS
    scripts/move_legs
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
