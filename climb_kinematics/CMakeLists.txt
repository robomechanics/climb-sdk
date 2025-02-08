cmake_minimum_required(VERSION 3.8)
project(climb_kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(climb_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(urdf REQUIRED)
find_package(climb_util REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/interfaces/kinematics_interface.cpp
  src/interfaces/kdl_interface.cpp
  src/kinematics_node.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen orocos-kdl)
ament_target_dependencies(${PROJECT_NAME} kdl_parser sensor_msgs geometry_msgs
  climb_msgs urdf tf2 tf2_ros climb_util)

ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(rclcpp std_msgs geometry_msgs sensor_msgs
  climb_msgs tf2_ros Eigen3 orocos_kdl kdl_parser urdf climb_util)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(DIRECTORY include/
  DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(${PROJECT_NAME}_test test/kinematics_test.cpp)
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
  ament_target_dependencies(${PROJECT_NAME}_test ament_index_cpp)

  install(DIRECTORY test/resources/
    DESTINATION share/${PROJECT_NAME}/test/resources
  )
endif()

ament_package()
