cmake_minimum_required(VERSION 3.20)
project(robot_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

# Generates C++ and Python code (and type support) for each interface.
# Interfaces must live in a package built with ament_cmake, even when every
# node that uses them is written in Python.
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/NavigateToWaypoints.action"
  "srv/SetSpeedLimit.srv"
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
