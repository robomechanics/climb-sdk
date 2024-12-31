cmake_minimum_required(VERSION 3.8)
project(climb_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(climb_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(climb_util REQUIRED)
find_package(climb_kinematics REQUIRED)
find_package(climb_optimization REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/contact_estimator.cpp
  src/force_estimator.cpp
  src/force_controller.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
ament_target_dependencies(${PROJECT_NAME} climb_util climb_kinematics
  climb_optimization climb_msgs geometry_msgs sensor_msgs)

add_executable(controller_node src/controller_node.cpp)
ament_target_dependencies(controller_node rclcpp sensor_msgs geometry_msgs
  climb_msgs std_srvs)
target_link_libraries(controller_node ${PROJECT_NAME})

install(TARGETS controller_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(climb_util climb_kinematics climb_optimization
  climb_msgs geometry_msgs sensor_msgs Eigen3)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE) # skips copyright check
  set(ament_cmake_cpplint_FOUND TRUE)   # skips cpplint
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(${PROJECT_NAME}_test test/controller_test.cpp)
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})

  install(DIRECTORY test/resources/
    DESTINATION share/${PROJECT_NAME}/test/resources
  )
endif()

ament_package()
