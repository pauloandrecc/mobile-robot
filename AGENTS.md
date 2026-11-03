# AGENTS.md

Guidance for AI coding agents working in this repository.

## Purpose of the project

This is a **learning project**: the owner is practicing ROS 2 after studying the
official tutorials. It is "Project 1 — mobile ground robot with autonomous
navigation" of a 3-project roadmap (mobile robot → drone → robotic arm). Each
phase is meant to exercise specific ROS 2 topics and robotics concepts, so when
helping, explain the *why* (ROS 2 concepts, robotics theory) and not only the fix.

The roadmap below comes from the original planning conversation. The plan was
written for Gazebo Classic; this repo uses modern Gazebo (see Environment), so
plugin names in the plan (`libgazebo_ros_*`) map to `gz-sim-*-system` plugins.

## Roadmap and status

Legend: ✅ done · 🟡 partially done · ⬜ not started

### Phase 1 — Workspace and robot model ✅
Topics: packages, build (colcon), rosdep, URDF, RViz.
Differential robot (chassis, 2 drive wheels, caster) with correct inertias,
published with `robot_state_publisher`, viewed in RViz (RobotModel, TF).
Robotics: links/joints, inertia tensor, TF tree derived from the URDF.
Package names differ from the plan: `robot_description`, `robot_bringup`
(plan: `my_robot_description`, `my_robot_bringup`).

### Phase 2 — Gazebo simulation 🟡
Topics: Gazebo, plugins, TF2, topics, nodes, rqt.
Done: world SDF, diff drive plugin (`/cmd_vel` → `/odom`), 2D Lidar (`/scan`),
joint state publisher, `ros_gz_bridge`, `simulation.launch.py`, teleop
(keyboard and gamepad via `joy` + `teleop_twist_joy`).
Missing from the plan: **IMU sensor** (and the camera mentioned in the project
overview). Validation with `rqt_graph` / `rqt_topic` / `ros2 topic hz /scan`.

### Phase 3 — SLAM and mapping 🟡
Topics: slam_toolbox, TF2 (map→odom→base_link), bag, parameters,
launch (composable nodes).
Done: `slam.launch.py` + `slam_toolbox_params.yaml`, live map in RViz
(`rviz/slam.rviz`), map saving with `map_saver_cli`.
Missing: **record a full mapping bag** (`ros2 bag record`) and **replay it
without Gazebo** to rebuild the map from recorded data; composable nodes.
The map in `maps/` was recorded before the URDF/SLAM fixes and must be redone.
Robotics: probabilistic occupancy grid, loop closure, why `map→odom` is
separate from `odom→base_link`.

### Phase 4 — Custom interfaces and navigation action server ⬜
Topics: msg, srv, interfaces, actions, services, parameters.
- New package `my_robot_interfaces` (or `robot_interfaces`) with
  `action/NavigateToWaypoints.action`:
  ```
  # Goal
  geometry_msgs/PoseStamped[] waypoints
  float64 max_speed
  ---
  # Result
  bool success
  float64 total_distance_m
  string[] visited_waypoints
  ---
  # Feedback
  string current_waypoint
  float64 distance_remaining
  uint8 percent_complete
  ```
- Action server that iterates over the waypoints calling Nav2 for each one and
  publishes feedback every second; a command-line action client to test it.
- Service `/set_speed_limit` that changes the max speed parameter at runtime
  (via `rcl_interfaces`).
- Planned package for these nodes: `my_robot_nav` (`waypoint_action_server.py`,
  `navigation_client.py`).
Note: the action server depends on Nav2, so parts of Phase 5 may be needed
first.

### Phase 5 — Full autonomous navigation with Nav2 ⬜
Topics: Nav2, launch (arguments and conditionals), parameters (yaml),
costmap, lifecycle nodes.
`config/nav2_params.yaml` (costmap resolution, inflation, DWB max speeds, goal
tolerances). Main `navigation.launch.py` with args `use_sim_time`,
`slam:=true/false` (SLAM or saved map + AMCL) and `rviz:=true/false`.
Scenarios: mapping (SLAM on, exploring) and navigation on a known map (SLAM off,
AMCL localizing).
Robotics: DWA (Dynamic Window Approach) and MPPI.

### Phase 6 — Tests and quality ⬜
Topics: tests (gtest, pytest, launch_testing), rosdep, CI.
- Unit tests (gtest/pytest) for isolated math, e.g. odometry computation.
- Integration test with `launch_testing`: start Gazebo, send a navigation goal,
  check the robot arrives within 0.3 m in under 60 s.
- Interface tests checking the custom messages' fields.
- All test dependencies declared in `package.xml`; simple CI running
  `colcon test` after the build.

### Extra topics suggested for the roadmap
ros2_control, Nav2 behavior trees, lifecycle nodes, composable nodes,
MCAP/rosbag2 offline analysis with Python, micro-ROS, SROS2,
`diagnostic_updater`.

## Environment

- Ubuntu 26.04, **ROS 2 Lyrical** (migrated from Kilted), **Gazebo Jetty**
  (gz-sim 10) via `ros_gz`.
- The owner uses **zsh**: source `install/setup.zsh`, never `setup.bash`.
- Gamepad: GameSir Nova Lite (2.4 GHz receiver, shows up as
  "Zikway HID gamepad"), read with `joy`'s `game_controller_node`
  (SDL standard layout: LB = 9, RB = 10, left stick = axes 0/1).

## Conventions and gotchas

- Many existing files use **CRLF line endings** (launch files, xacro, SDF,
  READMEs, slam params). Preserve the existing line endings when editing,
  otherwise the whole file shows up as changed in the diff.
- Docs and code comments are in English.
- `IncludeLaunchDescription` launch arguments leak into the including file;
  wrap includes in `GroupAction(..., scoped=True)` when the parent declares an
  argument with the same name (e.g. `use_rviz`).
- `slam_toolbox` 2.10 is a lifecycle node: `slam.launch.py` configures and
  activates it with lifecycle events. Do **not** use `nav2_lifecycle_manager`
  for it (no bond → manager aborts; `use_lifecycle_manager: true` → crash with
  `bad_weak_ptr`).
- URDF physics: wheel joints use `rpy="-pi/2 0 0"` (axis +Y), wheel collisions
  are spheres (cylinders contact at the inner edge and break odometry), and
  `base_height` keeps wheels and caster exactly on the ground (otherwise the
  robot tilts and the Lidar sees the floor).

## Verifying changes without a GUI

Run simulations headless and isolated so they don't interfere with a session
the owner may have open:

```bash
export ROS_DOMAIN_ID=<unused id> GZ_PARTITION=<unique name>
P=$(ros2 pkg prefix robot_description)/share/robot_description
ros2 launch robot_bringup slam.launch.py use_rviz:=false use_joystick:=false \
  world:="-s -r --headless-rendering $P/worlds/obstacles_world.sdf"
```

Compare odometry/SLAM against ground truth with `gz model -m mobile_robot -p`.
Build into a scratch directory (`--build-base/--install-base/--log-base`) to
avoid touching the owner's `build/` and `install/`.
