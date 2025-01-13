cmake_minimum_required(VERSION 3.8)
project(climb_optimization)

option(BUILD_OSQP "Build the OSQP solver interface" ON)
option(BUILD_PIQP "Build the PIQP solver interface" OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(climb_util REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/qp_interfaces/qp_interface.cpp
  src/qp_problem.cpp
)
ament_target_dependencies(${PROJECT_NAME} climb_util)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
if(BUILD_OSQP)
  find_package(osqp REQUIRED)
  target_sources(${PROJECT_NAME} PRIVATE src/qp_interfaces/osqp_interface.cpp)
  target_link_libraries(${PROJECT_NAME} osqp::osqp)
  ament_export_dependencies(osqp)
  target_compile_definitions(${PROJECT_NAME} PUBLIC BUILT_OSQP)
endif()
if(BUILD_PIQP)
  find_package(piqp REQUIRED)
  target_sources(${PROJECT_NAME} PRIVATE src/qp_interfaces/piqp_interface.cpp)
  target_link_libraries(${PROJECT_NAME} piqp::piqp)
  ament_export_dependencies(piqp)
  target_compile_definitions(${PROJECT_NAME} PUBLIC BUILT_PIQP)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 climb_util)
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
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(${PROJECT_NAME}_test test/optimization_test.cpp)
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
endif()

ament_package()
