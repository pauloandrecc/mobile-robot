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

### Phase 2 — Gazebo simulation ✅
Topics: Gazebo, plugins, TF2, topics, nodes, rqt.
Done: world SDF, diff drive plugin (`/cmd_vel` → `/odom`), 2D Lidar (`/scan`),
IMU (`/imu`, needs `gz-sim-imu-system` in the world), camera
(`/camera/image_raw`, `/camera/camera_info`, optical frame
`camera_optical_link`), joint state publisher, `ros_gz_bridge`,
`simulation.launch.py`, teleop (keyboard and gamepad via `joy` +
`teleop_twist_joy`). Validation commands (`ros2 topic hz`, `rqt_graph`,
`rqt_topic`, `rqt_image_view`, `rqt_plot`) documented in the README.
Measured rates (headless, RTF ~1.0): scan ~10 Hz, imu 100 Hz, odom 20 Hz,
camera ~13 Hz.

### Phase 3 — SLAM and mapping ✅
Topics: slam_toolbox, TF2 (map→odom→base_link), bag, parameters,
launch (composable nodes).
Done: `slam.launch.py` + `slam_toolbox_params.yaml`, live map in RViz
(`rviz/slam.rviz`), map saving with `map_saver_cli`, bag recording
(`slam.launch.py record:=true`, MCAP, sim time), offline replay without
Gazebo (`slam_replay.launch.py` + `scripts/bag_tf_filter.py`, which keeps the
recorded map→odom out of /tf), composable nodes (gamepad container in
`joystick.launch.py`). slam_toolbox startup is shared in
`slam_toolbox.launch.py`. Replayed map matched the live one in 99.5% of
cells.
Robotics: probabilistic occupancy grid, loop closure, why `map→odom` is
separate from `odom→base_link`.

### Phase 4 — Custom interfaces and navigation action server ✅
Topics: msg, srv, interfaces, actions, services, parameters.
Done: package `robot_interfaces` (ament_cmake) with
`action/NavigateToWaypoints.action` (goal: `PoseStamped[] waypoints`,
`float64 max_speed`; result: `success`, `total_distance_m`,
`visited_waypoints`; feedback: `current_waypoint`, `distance_remaining`,
`percent_complete`) and `srv/SetSpeedLimit.srv`. Package `robot_navigation`
(ament_python; plan name was `my_robot_nav`):
`waypoint_action_server` (action server `/navigate_to_waypoints` that is also
a client of Nav2's `navigate_to_pose`, one goal per waypoint, feedback every
1 s, cancel forwarded to Nav2, MultiThreadedExecutor + ReentrantCallbackGroup),
`navigation_client` (CLI, Ctrl+C cancels), service `/set_speed_limit` that
sets the `max_speed` parameter (range-checked via `ParameterDescriptor`,
returns `rcl_interfaces/SetParametersResult`) and publishes
`nav2_msgs/SpeedLimit` on `/speed_limit`. Pure math in `waypoint_math.py`
with pytest unit tests. Pulled forward from Phase 5: minimal
`config/nav2_params.yaml` (DWB inside RotationShimController, NavFn, costmaps
on `base_footprint`) and `navigation.launch.py` (slam.launch.py + 4 Nav2
servers + lifecycle manager + waypoint server).
Verified headless: 3-waypoint route around box_1 in 35 s, final pose 0.23 m /
12° from the goal (tolerances 0.25 m / 0.25 rad); speed 0.30 → 0.09 m/s after
`/set_speed_limit 0.1`; Ctrl+C → CANCELED and the robot stops.

### Phase 5 — Full autonomous navigation with Nav2 ✅
Topics: Nav2, launch (arguments and conditionals), parameters (yaml),
costmap, lifecycle nodes.
Done: `navigation.launch.py` with `use_sim_time`, `slam:=true/false`,
`map:=` (default `./maps/my_map.yaml`), `controller:=dwb/mppi` and
`use_rviz` (plan said `rviz:=`; kept `use_rviz` like the other launch
files). `slam:=true` includes `slam_toolbox.launch.py`; `slam:=false` starts
`map_server` + `amcl` + `lifecycle_manager_localization` (AMCL seeded at
0,0,0 = spawn pose = origin of maps saved from there). Controller split out
of `nav2_params.yaml` into `config/controller_dwb.yaml` (RotationShim + DWB)
and `config/controller_mppi.yaml`; `controller_server` loads both files,
last one wins. RViz view `robot_description/rviz/nav2.rviz` (Nav2 panel,
Nav2 Goal, 2D Pose Estimate, costmaps, plans, MPPI trajectories, AMCL
particles). Tuning measured on the README route
(2.5,0 → 3.2,2,90° → 0,0,180°): inflation 0.45 m/k=3 → 0.6 m/k=4 and goal
tolerance 0.25 → 0.15 (m and rad). Results (ground truth):
SLAM+DWB 37.5 s, 0.14 m/6°; AMCL+DWB 37 s, 0.12 m/7°; AMCL+MPPI 35 s,
0.14 m/5°; AMCL vs ground truth ≤ 3.5 cm. Phase 4 values gave 39 s,
0.23 m/10°; inflation 0.8 m gave 53 s (wide detours around box_1).
Robotics: DWA (Dynamic Window Approach) and MPPI, AMCL (particle filter),
costmap inflation.
Not done (optional): `robot_localization` EKF fusing odom + IMU,
velocity_smoother / collision_monitor, keepout or speed filter zones.

### Phase 6 — Tests and quality ✅
Topics: tests (gtest, pytest, launch_testing), rosdep, CI.
Done (all pytest; there is no C++ code, so no gtest):
- Unit: `robot_navigation/diff_drive_kinematics.py` (wheel speeds ↔ twist,
  exact arc vs Euler odometry integration) + `test_diff_drive_kinematics.py`;
  `robot_description/test/test_urdf.py` (TF tree single root
  `base_footprint`, inertia triangle inequality, wheels/caster touching the
  ground, DiffDrive `wheel_separation`/`wheel_radius` = URDF geometry).
  Mutation check: plugin separation 0.26 → 0.25 makes it fail.
- Interfaces: `robot_interfaces/test/test_interfaces.py` (fields/types,
  defaults, `uint8` bounds with `check_fields=True`, serialization).
- Integration: `robot_bringup/test/test_navigation.launch.py`
  (`add_launch_test`, label `simulation`, `run_test_isolated.py` runner for
  a private ROS_DOMAIN_ID, own `GZ_PARTITION`). Starts
  `navigation.launch.py slam:=true` headless, waits for
  `/lifecycle_manager_navigation/is_active`, sends (3.2, 2.0, 90°) behind
  box_1 via `/navigate_to_waypoints`, checks Gazebo ground truth
  (`gz model -p`) < 0.3 m and < 60 s sim time; post-shutdown checks the
  waypoint server exits with 0. Measured: 14.3–14.5 s sim, 0.135–0.153 m,
  ~40 s wall per run; 1 of 5 runs hit the lost lifecycle response.
- Test deps declared in each `package.xml`; CI in
  `.github/workflows/ci.yml` (`ros:lyrical-ros-base` container, rosdep
  resolves every key, installs only build+test deps, `colcon test
  --ctest-args -LE simulation`). Verified locally with docker: 99 tests,
  0 failures.

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
- The gz-sim `DiffDrive` system has no covariance parameters: `/odom` is
  published with all-zero covariance ("infinite confidence"). Before fusing
  odometry with the IMU (e.g. `robot_localization` in Phase 5), add realistic
  covariances (for example with a small relay node).
- World materials need `<diffuse>` (not only `<ambient>`): the ogre2 PBR
  renderer uses mostly diffuse, so ambient-only objects render black.
- Nav2 1.5 (`ros-lyrical-navigation2`): `controller_server` and
  `behavior_server` publish `TwistStamped` unless
  `enable_stamped_cmd_vel: false`; the Gazebo bridge only carries `Twist` on
  `/cmd_vel`, so without it the robot silently never moves.
- Since Nav2 Kilted, `RotationShimController` takes the primary controller as
  a namespace (`FollowPath.primary_controller.plugin` + its params under it),
  not as a plugin-name string. Plain DWB stalls ("Failed to make progress")
  when the next waypoint is behind the robot; the shim fixes it.
- Waypoint goals are sent with stamp 0 (latest TF). Stamping with a
  wall-clock node's time makes Nav2 (sim time) look up TF far in the future.
- Interrupting a headless `gz sim` launch can leave the `gz-sim-main` server
  orphaned and still publishing `/clock` and `/tf` in that `GZ_PARTITION`
  (symptom on the next run: sim time not starting at 0, `TF_OLD_DATA`
  warnings). Check for leftover `gz-sim-main` processes before relaunching.
  Cause: ros_gz_sim runs `gz sim` (Ruby wrapper) which forks
  `gz-sim-main`; on shutdown only the wrapper gets the signal. This
  happens on every normal launch_testing shutdown too; the launch test
  kills its own server at exit, found by `GZ_PARTITION` in
  `/proc/<pid>/environ`.
- `TimerAction` runs its actions later, after an enclosing `GroupAction`'s
  scope has already been popped. Put the scoped `GroupAction` (and any
  `SetParameter`) *inside* the timer, never around it.
- Under load (owner's Gazebo GUI + a headless test, load avg ~10), a
  lifecycle `change_state` response is sometimes lost
  (`failed to send response to /<node>/change_state (timeout)` in the log):
  the node is configured but `lifecycle_manager` waits forever and Nav2
  never becomes active. Happened in 3 of 12 test launches; relaunching
  fixes it. Not caused by the config. Also seen with low load (~1.5) in
  the launch test, so it looks like a DDS discovery race rather than load;
  `lifecycle_manager`'s `service_timeout` does not bound the wait for the
  transition response. The launch test fails after 60 s; rerun it with
  `--ctest-args --repeat until-pass:3`.
- When killing test launches, `pkill -f '<pattern>'` also matches (and
  kills) the shell running it; use `'[p]attern'`. SIGINT does not always
  stop every node of a `ros2 launch`; `kill -9` on DDS nodes leaves stale
  Fast DDS shared memory (clean with `fastdds shm clean`).
- `colcon test` runs `ament_lint_common` (flake8, pep257, xmllint,
  lint_cmake); keep it at 0 failures.
- Generated Python messages do **not** check field types/ranges by default
  (since Jazzy): `uint8 = 300` is accepted and serialized as 44. Checks
  are on with `Msg(check_fields=True)` or `ROS_PYTHON_CHECK_FIELDS=1`.
- `colcon` global options go before the verb: `colcon --log-base X build
  --build-base Y --install-base Z` (and `colcon test-result
  --test-result-base Y`).

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
