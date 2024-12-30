cmake_minimum_required(VERSION 3.8)
project(loris_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(climb_sdk REQUIRED)

# Add tests and test dependencies
if(BUILD_TESTING)
find_package(ament_lint_auto REQUIRED)
set(ament_cmake_copyright_FOUND TRUE) # skips copyright check
set(ament_cmake_cpplint_FOUND TRUE)   # skips cpplint
ament_lint_auto_find_test_dependencies()
endif()

# Install ROS launch and config files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Finalize build
ament_package()
