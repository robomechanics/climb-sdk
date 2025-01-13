cmake_minimum_required(VERSION 3.8)
project(climb_gait_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(climb_msgs REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(climb_util REQUIRED)
find_package(climb_kinematics REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/gait_planner.cpp
)
ament_target_dependencies(${PROJECT_NAME} climb_util climb_kinematics
  geometry_msgs climb_msgs)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)

add_executable(gait_planner_node src/gait_planner_node.cpp)
ament_target_dependencies(gait_planner_node rclcpp rclcpp_action geometry_msgs
  climb_msgs)
target_link_libraries(gait_planner_node ${PROJECT_NAME})

install(TARGETS
  gait_planner_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(${PROJECT_NAME}_test test/gait_planner_test.cpp)
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})

  install(DIRECTORY test/resources/
    DESTINATION share/${PROJECT_NAME}/test/resources
  )
endif()

ament_package()
