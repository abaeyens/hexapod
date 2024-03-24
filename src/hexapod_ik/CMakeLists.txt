cmake_minimum_required(VERSION 3.8)
project(hexapod_ik)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(std_msgs REQUIRED)

set(dependencies
  rclcpp
  kdl_parser
  std_msgs
)

add_executable(leg_inverse
  src/leg_inverse.cpp
)
target_include_directories(leg_inverse PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(leg_inverse ${dependencies})

install(TARGETS
  leg_inverse
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
