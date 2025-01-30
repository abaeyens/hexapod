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
find_package(urdf REQUIRED)

set(dependencies
  rclcpp
  orocos_kdl_vendor
  kdl_parser
  std_msgs
  urdf
)

#########
# build #
#########
# library
add_library(${PROJECT_NAME}
  src/kinematics.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${dependencies})

# example
add_executable(example
  src/example.cpp
)
target_link_libraries(example ${PROJECT_NAME})
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
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})

# example
install(TARGETS
  example
  DESTINATION lib/${PROJECT_NAME}
)

########
# test #
########
if (BUILD_TESTING)
  # URDF file used in tests
  install(FILES
    test/robot_description.urdf
    DESTINATION share/${PROJECT_NAME}/test
  )

  # C++ unit tests
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_index_cpp REQUIRED)
  set(test_dependencies
    ament_index_cpp
  )
  ament_add_gtest(test_kinematics test/test_kinematics.cpp)
  target_link_libraries(test_kinematics ${PROJECT_NAME})
  ament_target_dependencies(
      test_kinematics ${dependencies} ${test_dependencies})
endif()

ament_package()
