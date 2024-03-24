cmake_minimum_required(VERSION 3.8)
project(hexapod_kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(orocos_kdl_vendor REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(std_msgs REQUIRED)

set(dependencies
  rclcpp
  orocos_kdl_vendor
  kdl_parser
  std_msgs
)

#########
# build #
#########
# library
add_library(kinematics
  src/kinematics.cpp
)
target_include_directories(kinematics PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
ament_target_dependencies(kinematics PUBLIC ${dependencies})

# example
# TODO: do this using the above library instead of adding the sources a second time?
add_executable(example
  src/example.cpp
  src/kinematics.cpp
)
target_include_directories(example PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(example ${dependencies})

###########
# install #
###########
# library
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS kinematics
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(dependencies)

# example
install(TARGETS
  example
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
