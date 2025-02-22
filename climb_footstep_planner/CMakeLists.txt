cmake_minimum_required(VERSION 3.8)
project(climb_footstep_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(climb_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(ompl REQUIRED)
find_package(climb_util REQUIRED)
find_package(climb_kinematics REQUIRED)
# find_package(climb_control REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/planners/planner.cpp
  src/planners/local_planner.cpp
  src/planners/global_planner.cpp
  src/terrain_generator.cpp
)
ament_target_dependencies(${PROJECT_NAME} climb_util climb_kinematics
  rclcpp climb_msgs sensor_msgs geometry_msgs tf2_ros pcl_ros)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen /opt/ros/jazzy/lib/x86_64-linux-gnu/libompl.so)
include_directories(/opt/ros/jazzy/include/ompl-1.6)

add_executable(footstep_planner_node src/footstep_planner_node.cpp)
ament_target_dependencies(footstep_planner_node rclcpp climb_msgs nav_msgs
  sensor_msgs geometry_msgs std_msgs visualization_msgs std_srvs tf2_ros
  pcl_ros pcl_conversions)
target_link_libraries(footstep_planner_node ${PROJECT_NAME})

install(TARGETS
  footstep_planner_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(${PROJECT_NAME}_test test/footstep_planner_test.cpp)
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
endif()

ament_package()
