cmake_minimum_required(VERSION 3.8)
project(climb_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/FootstepCommand.action"
  "msg/ActuatorState.msg"
  "msg/ContactForce.msg"
  "msg/ControllerCommand.msg"
  "msg/Footstep.msg"
  "msg/FootstepPlan.msg"
  "msg/FootstepUpdate.msg"
  "msg/JointCommand.msg"
  "msg/TeleopOutput.msg"
  "srv/ActuatorEnable.srv"
  "srv/SetString.srv"
  "srv/TeleopInput.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
