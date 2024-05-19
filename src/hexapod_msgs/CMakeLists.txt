cmake_minimum_required(VERSION 3.8)
project(hexapod_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

##############
# interfaces #
##############
set(dependencies
  std_msgs
  geometry_msgs
)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Vector3Array.msg"
  "msg/GaitPhase.msg"
  DEPENDENCIES ${dependencies}
)

ament_package()
