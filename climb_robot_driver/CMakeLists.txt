cmake_minimum_required(VERSION 3.8)
project(climb_robot_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(climb_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(climb_util REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/interfaces/hardware_interface.cpp
  src/interfaces/dynamixel_interface.cpp
  src/interfaces/dummy_interface.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME} dynamixel_sdk climb_msgs
  sensor_msgs climb_util)

add_executable(robot_driver_node src/robot_driver_node.cpp)
ament_target_dependencies(robot_driver_node rclcpp sensor_msgs climb_msgs)
target_link_libraries(robot_driver_node ${PROJECT_NAME})

install(TARGETS robot_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(${PROJECT_NAME}_test test/hardware_interface_test.cpp)
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
endif()

ament_package()
