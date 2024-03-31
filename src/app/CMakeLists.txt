cmake_minimum_required(VERSION 3.8)
project(app)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(actuator_msgs REQUIRED)
find_package(orocos_kdl_vendor REQUIRED)
find_package(hexapod_kinematics REQUIRED)

set(dependencies
  rclcpp
  std_msgs
  actuator_msgs
  orocos_kdl_vendor
  hexapod_kinematics
)

#########
# build #
#########
add_executable(walk_forward
  src/walk_forward.cpp
)
target_include_directories(walk_forward PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(walk_forward ${dependencies})

###########
# install #
###########
install(TARGETS
  walk_forward
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/move_legs
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
