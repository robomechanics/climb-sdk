cmake_minimum_required(VERSION 3.8)
project(climb_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(climb_msgs REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(keyboard_handler REQUIRED)
find_package(fmt REQUIRED)
find_package(climb_util REQUIRED)
find_package(climb_kinematics REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/key_input_parser.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(teleop_node src/teleop_node.cpp)
ament_target_dependencies(teleop_node rclcpp rclcpp_action climb_msgs
  climb_util climb_kinematics)
target_include_directories(teleop_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(teleop_node ${PROJECT_NAME} fmt::fmt)

add_executable(key_input_node src/key_input_node.cpp)
ament_target_dependencies(key_input_node rclcpp climb_msgs keyboard_handler)
target_include_directories(key_input_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS teleop_node key_input_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE) # skips copyright check
  set(ament_cmake_cpplint_FOUND TRUE)   # skips cpplint
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
