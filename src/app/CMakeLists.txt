cmake_minimum_required(VERSION 3.8)
project(app)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(actuator_msgs REQUIRED)
find_package(orocos_kdl_vendor REQUIRED)
find_package(hexapod_kinematics REQUIRED)
find_package(hexapod_msgs REQUIRED)

set(dependencies
  rclcpp
  std_msgs
  geometry_msgs
  actuator_msgs
  orocos_kdl_vendor
  hexapod_kinematics
  hexapod_msgs
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

add_executable(follow_velocity_rectangle
  src/follow_velocity_rectangle.cpp
)
target_include_directories(follow_velocity_rectangle PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(follow_velocity_rectangle ${dependencies})

###########
# install #
###########
install(TARGETS
  walk_forward
  follow_velocity_rectangle
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/move_legs
    scripts/walk_sideways_while_rotating
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
