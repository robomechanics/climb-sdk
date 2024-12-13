cmake_minimum_required(VERSION 3.8)
project(climb_main)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(climb_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(urdf REQUIRED)
find_package(osqp REQUIRED)
find_package(piqp REQUIRED)
find_package(keyboard_handler REQUIRED)
find_package(fmt REQUIRED)

# Utilities library
add_library(${PROJECT_NAME}_util SHARED
  src/util/parameterized.cpp
  src/util/eigen_utils.cpp
)
target_include_directories(${PROJECT_NAME}_util PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_util Eigen3::Eigen)
ament_target_dependencies(${PROJECT_NAME}_util rclcpp geometry_msgs)

# Hardware library
add_library(${PROJECT_NAME}_hardware SHARED
  src/hardware/hardware_interface.cpp
  src/hardware/dynamixel_interface.cpp
  src/hardware/dummy_interface.cpp
)
target_include_directories(${PROJECT_NAME}_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_hardware ${PROJECT_NAME}_util)
ament_target_dependencies(${PROJECT_NAME}_hardware dynamixel_sdk climb_msgs
  sensor_msgs)

# Kinematics library
add_library(${PROJECT_NAME}_kinematics SHARED
  src/kinematics/kinematics_interface.cpp
  src/kinematics/kdl_interface.cpp
  src/kinematics/kinematics_node.cpp
)
target_include_directories(${PROJECT_NAME}_kinematics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_kinematics ${PROJECT_NAME}_util
  Eigen3::Eigen orocos-kdl)
ament_target_dependencies(${PROJECT_NAME}_kinematics kdl_parser sensor_msgs
  geometry_msgs climb_msgs urdf tf2 tf2_ros)

# Optimization library
add_library(${PROJECT_NAME}_optimization SHARED
  src/optimization/qp_interface.cpp
  src/optimization/osqp_interface.cpp
  src/optimization/piqp_interface.cpp
  src/optimization/qp_problem.cpp
)
target_include_directories(${PROJECT_NAME}_optimization PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_optimization ${PROJECT_NAME}_util
  osqp::osqp Eigen3::Eigen piqp::piqp)

# Controller library
add_library(${PROJECT_NAME}_controller SHARED
  src/controller/contact_estimator.cpp
  src/controller/force_controller.cpp
  src/controller/force_estimator.cpp
  src/controller/contact_estimator.cpp
)
target_include_directories(${PROJECT_NAME}_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_controller ${PROJECT_NAME}_kinematics
  ${PROJECT_NAME}_optimization ${PROJECT_NAME}_util Eigen3::Eigen)
ament_target_dependencies(${PROJECT_NAME}_controller climb_msgs sensor_msgs)

# Step planner library
add_library(${PROJECT_NAME}_step_planner SHARED
  src/step_planner/step_planner.cpp
)
target_include_directories(${PROJECT_NAME}_step_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_step_planner ${PROJECT_NAME}_kinematics
  ${PROJECT_NAME}_util Eigen3::Eigen)

# Teleop library
add_library(${PROJECT_NAME}_teleop SHARED
  src/teleop/key_input_parser.cpp
)
target_include_directories(${PROJECT_NAME}_teleop PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Hardware node executable
add_executable(hardware_node src/hardware_node.cpp)
ament_target_dependencies(hardware_node rclcpp sensor_msgs climb_msgs)
target_link_libraries(hardware_node ${PROJECT_NAME}_hardware)

# Controller node executable
add_executable(controller_node src/controller_node.cpp)
ament_target_dependencies(controller_node rclcpp std_msgs sensor_msgs
  geometry_msgs climb_msgs)
target_link_libraries(controller_node ${PROJECT_NAME}_controller)

# Step planner node executable
add_executable(step_planner_node src/step_planner_node.cpp)
ament_target_dependencies(step_planner_node rclcpp rclcpp_action climb_msgs
  sensor_msgs)
target_link_libraries(step_planner_node ${PROJECT_NAME}_step_planner)

# Teleop node executable
add_executable(teleop_node src/teleop_node.cpp)
ament_target_dependencies(teleop_node rclcpp rclcpp_action sensor_msgs
  climb_msgs)
target_include_directories(teleop_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(teleop_node ${PROJECT_NAME}_kinematics
  ${PROJECT_NAME}_teleop fmt::fmt)

# Key input node executable
add_executable(key_input_node src/key_input_node.cpp)
ament_target_dependencies(key_input_node rclcpp climb_msgs keyboard_handler)
target_include_directories(key_input_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install ROS nodes
install(TARGETS
  hardware_node
  controller_node
  step_planner_node
  teleop_node
  key_input_node
  DESTINATION lib/${PROJECT_NAME})

# Add tests and test dependencies
if(BUILD_TESTING)
  # Linting tests
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Utilities tests
  ament_add_gtest(${PROJECT_NAME}_util_test test/util_test.cpp)
  target_link_libraries(${PROJECT_NAME}_util_test ${PROJECT_NAME}_util)
  target_include_directories(${PROJECT_NAME}_util_test PRIVATE include)

  # Hardware tests
  ament_add_gtest(${PROJECT_NAME}_hardware_test test/hardware_test.cpp)
  target_link_libraries(${PROJECT_NAME}_hardware_test ${PROJECT_NAME}_hardware)
  target_include_directories(${PROJECT_NAME}_hardware_test PRIVATE include
    test/util)

  # Kinematics tests
  ament_add_gtest(${PROJECT_NAME}_kinematics_test test/kinematics_test.cpp)
  target_link_libraries(${PROJECT_NAME}_kinematics_test
    ${PROJECT_NAME}_kinematics)
  target_include_directories(${PROJECT_NAME}_kinematics_test PRIVATE include
    test/util)

  # Controller tests
  ament_add_gtest(${PROJECT_NAME}_controller_test test/controller_test.cpp)
  target_link_libraries(${PROJECT_NAME}_controller_test
    ${PROJECT_NAME}_controller)
  target_include_directories(${PROJECT_NAME}_controller_test PRIVATE include
    test/util)

  # Optimization tests
  ament_add_gtest(${PROJECT_NAME}_optimization_test test/optimization_test.cpp)
  target_link_libraries(${PROJECT_NAME}_optimization_test
    ${PROJECT_NAME}_optimization)
  target_include_directories(${PROJECT_NAME}_optimization_test PRIVATE include
    test/util)

  # Step planner tests
  ament_add_gtest(${PROJECT_NAME}_step_planner_test test/step_planner_test.cpp)
  target_link_libraries(${PROJECT_NAME}_step_planner_test
    ${PROJECT_NAME}_step_planner)
  target_include_directories(${PROJECT_NAME}_step_planner_test PRIVATE include
    test/util)

  install(DIRECTORY test/resources/
    DESTINATION share/${PROJECT_NAME}/test/resources
  )
endif()

# Install ROS launch and config files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Finalize build
ament_package()
