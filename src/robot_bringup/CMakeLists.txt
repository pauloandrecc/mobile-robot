cmake_minimum_required(VERSION 3.20)
project(robot_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Integration test: headless Gazebo + SLAM + Nav2 + waypoint server, one
  # goal, checked against Gazebo's ground truth. Needs a machine that can
  # run Gazebo, so it is labeled "simulation":
  #   colcon test --ctest-args -L simulation    # only this test
  #   colcon test --ctest-args -LE simulation   # everything else (CI)
  # The isolated runner gives the test its own ROS_DOMAIN_ID, so it does
  # not talk to a simulation the user may have open.
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(ament_cmake_ros REQUIRED)
  add_launch_test(test/test_navigation.launch.py
    TIMEOUT 300
    LABELS simulation
    RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py"
  )
endif()

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# Python nodes (run with: ros2 run robot_bringup <script>.py)
install(PROGRAMS
  scripts/bag_tf_filter.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
