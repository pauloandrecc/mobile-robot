# robot_bringup

Top-level launch files and configuration for the mobile robot.
This package orchestrates all other packages — simulation, SLAM, and navigation.

## Package structure

- robot_bringup/
- launch/
- simulation.launch.py      — Gazebo simulation with diff drive and sensors
- slam.launch.py            — SLAM mapping with slam_toolbox (+ bag recording)
- slam_replay.launch.py     — rebuild a map from a recorded bag, no Gazebo
- slam_toolbox.launch.py    — starts and activates slam_toolbox (shared)
- joystick.launch.py        — gamepad teleoperation (composable nodes)
- navigation.launch.py      — Nav2 on SLAM or on a saved map (AMCL) + waypoint server
- scripts/
- bag_tf_filter.py          — forwards only odom→base_footprint from a bag's /tf
- config/
- slam_toolbox_params.yaml  — slam_toolbox tuning parameters
- joystick.yaml             — gamepad button/axis mapping and speeds
- nav2_params.yaml          — Nav2 servers, costmaps, AMCL, map_server
- controller_dwb.yaml       — local controller: DWB inside a rotation shim
- controller_mppi.yaml      — local controller: MPPI

## Launch files

### simulation.launch.py

Brings up the full Gazebo simulation with the robot, sensor bridge and RViz.

```bash
ros2 launch robot_bringup simulation.launch.py
```

Arguments:

| Argument | Default | Description |
|----------|---------|-------------|
| `use_rviz` | `true` | Launch RViz alongside Gazebo |
| `use_joystick` | `true` | Start gamepad teleoperation (`joystick.launch.py`) |
| `world` | `obstacles_world.sdf` | Path to the Gazebo world file |
| `world_name` | `obstacles_world` | World name (used for joint state topic bridge) |

Example — launch with a custom world:

```bash
ros2 launch robot_bringup simulation.launch.py \
  world:=/path/to/world.sdf \
  world_name:=my_world
```

### joystick.launch.py

Starts `game_controller_node` from `joy` (reads the gamepad with SDL's
standard controller layout, publishes `/joy`) and
`teleop_twist_joy` (converts `/joy` into `/cmd_vel`). Included automatically
by `simulation.launch.py`; it can also be run on its own:

Both are loaded as **composable nodes** (components) into a single process,
`gamepad_container`, with intra-process communication: `/joy` messages are
passed by pointer instead of being serialized through DDS. Inspect it with
`ros2 component list`.

```bash
ros2 launch robot_bringup joystick.launch.py
```

| Argument | Default | Description |
|----------|---------|-------------|
| `joy_config` | `config/joystick.yaml` | Parameter file with mapping and speeds |
| `joy_dev` | `0` | Joystick device index |

Hold **LB** and use the left stick to drive; hold **RB** instead for turbo.

### slam.launch.py

Launches the full simulation and starts SLAM mapping with slam_toolbox.
slam_toolbox is a lifecycle node: the launch file configures and activates
it automatically (lifecycle events, same approach as slam_toolbox's own
launch files), and opens RViz with `robot_description/rviz/slam.rviz`.

```bash
ros2 launch robot_bringup slam.launch.py
```

Arguments:

| Argument | Default | Description |
|----------|---------|-------------|
| `use_sim_time` | `true` | Use simulation clock |
| `use_rviz` | `true` | Launch RViz with the SLAM view |
| `use_joystick` | `true` | Start gamepad teleoperation |
| `world` | `obstacles_world.sdf` | Path to the Gazebo world file |
| `world_name` | `obstacles_world` | World name (used for joint state topic bridge) |
| `record` | `false` | Record the session to a bag (MCAP) |
| `bag` | `bags/mapping_<date>` | Output folder of the bag, relative to the current directory |

With `record:=true` the topics `/scan /odom /tf /tf_static /joint_states
/imu /cmd_vel` are recorded with simulation timestamps (`--use-sim-time`).

> **Important:** After launching, click the **play button** in Gazebo and
> wait ~6 seconds before moving the robot. This gives the SLAM node time
> to initialize and all TF frames time to stabilize.

### slam_replay.launch.py

Rebuilds a map with slam_toolbox from a bag recorded by `slam.launch.py`,
**without Gazebo**: the bag provides the Lidar scans, the odometry and the
clock.

```bash
ros2 launch robot_bringup slam_replay.launch.py bag:=bags/mapping_<date>
```

| Argument | Default | Description |
|----------|---------|-------------|
| `bag` | — (required) | Bag folder to replay |
| `rate` | `1.0` | Playback speed |
| `use_rviz` | `true` | Launch RViz with the SLAM view |

How it works: the recorded `/tf` contains `odom→base_footprint` (wanted),
the wheel transforms and the **old SLAM's `map→odom`**, which would fight the
new SLAM. The bag is played with `/tf` remapped to `/tf_bag`, and
`bag_tf_filter.py` forwards only `odom→base_footprint` to `/tf`.
`robot_state_publisher` republishes the robot frames and the new
slam_toolbox publishes `map→odom`. `/clock` comes from the bag
(`ros2 bag play --clock`).

### navigation.launch.py

Starts the simulation, a localization source, then (after 8 s) the Nav2
servers used by the default NavigateToPose behavior tree
(`controller_server`, `planner_server`, `behavior_server`, `bt_navigator`),
a `nav2_lifecycle_manager` that configures/activates them,
`robot_navigation`'s `waypoint_action_server` and RViz with
`robot_description/rviz/nav2.rviz`.

Two scenarios:

| Scenario | Command | Who publishes `/map` and `map→odom` |
|----------|---------|-------------------------------------|
| Mapping | `slam:=true` (default) | slam_toolbox (map grows while driving) |
| Known map | `slam:=false` | `map_server` (saved map) + AMCL, managed by `lifecycle_manager_localization` |

```bash
# from the repository root (map:= defaults to ./maps/my_map.yaml)
ros2 launch robot_bringup navigation.launch.py                   # SLAM
ros2 launch robot_bringup navigation.launch.py slam:=false       # AMCL
ros2 launch robot_bringup navigation.launch.py slam:=false controller:=mppi
ros2 run robot_navigation navigation_client 2.5,0 3.2,2.0,90 0,0,180
```

| Argument | Default | Description |
|----------|---------|-------------|
| `use_sim_time` | `true` | Use simulation clock |
| `slam` | `true` | `true`: slam_toolbox; `false`: saved map + AMCL |
| `map` | `./maps/my_map.yaml` | Map loaded when `slam:=false` (relative to the current directory) |
| `controller` | `dwb` | Local controller: `dwb` or `mppi` (`config/controller_<name>.yaml`) |
| `use_rviz` | `true` | Launch RViz with the Nav2 view |
| `use_joystick` | `true` | Start gamepad teleoperation |
| `world` | `obstacles_world.sdf` | Path to the Gazebo world file |
| `nav2_params` | `config/nav2_params.yaml` | Nav2 parameters file |
| `max_speed` | `0.3` | Initial speed limit of the waypoint server (m/s) |

AMCL is seeded at (0, 0, 0): the robot spawns at the world origin, which is
also the origin of a map saved from a SLAM session started there. If the
robot starts elsewhere, use RViz's **2D Pose Estimate** tool.

Notes on the configuration:

- `enable_stamped_cmd_vel: false` — Nav2 1.5 publishes `TwistStamped` by
  default, but the Gazebo bridge expects `Twist` on `/cmd_vel`.
- `controller_server` gets two parameter files: `nav2_params.yaml` and
  `controller_<name>.yaml` (the last file wins), so only the `FollowPath`
  plugin changes between DWB and MPPI.
- DWB is wrapped in `RotationShimController`, which turns in place towards
  the path first (DWB alone stalls when the next waypoint is behind the
  robot). Since Nav2 Kilted, DWB's parameters live under
  `FollowPath.primary_controller`. MPPI needs no shim.
- `speed_limit_topic: /speed_limit` — the waypoint server publishes
  `nav2_msgs/SpeedLimit` there to cap the controller's speed at runtime
  (works with both controllers).
- Tuning (measured on the route above, AMCL): inflation 0.6 m /
  `cost_scaling_factor` 4.0 and goal tolerance 0.15 m / 0.15 rad. Final
  error went from 0.23 m / 10° (Phase 4 values) to 0.12-0.14 m / 5-7°, in
  the same time (~37 s DWB, ~35 s MPPI). A 0.8 m inflation made the route
  take 53 s (wide detours around box_1).

## SLAM workflow

1. Launch SLAM: `ros2 launch robot_bringup slam.launch.py`
2. Click play in Gazebo and wait 6 seconds
3. Teleoperate: hold **LB** on the gamepad and use the left stick
   (or `ros2 run teleop_twist_keyboard teleop_twist_keyboard`)
4. Explore the environment slowly — prefer straight lines over sharp turns
5. Save the map (from the repository root):
   `ros2 run nav2_map_server map_saver_cli -f maps/my_map`

To debug offline, record the session with `record:=true` and replay it
later with `slam_replay.launch.py`.

## ROS 2 topics

| Topic | Type | Description |
|-------|------|-------------|
| `/map` | `nav_msgs/OccupancyGrid` | Occupancy grid map built by SLAM |
| `/map_metadata` | `nav_msgs/MapMetaData` | Map resolution and origin |
| `/slam_toolbox/scan_visualization` | `sensor_msgs/LaserScan` | Scan as seen by SLAM |
| `/pose` | `geometry_msgs/PoseWithCovarianceStamped` | Robot pose estimate |

## TF tree (SLAM active)

map
└── odom
└── base_footprint
└── base_link
├── left_wheel
├── right_wheel
├── caster_wheel
└── lidar_link

## Known issues

- The slam_toolbox node must be started after the simulation clock is
  running — the 6-second delay in the launch file handles this automatically.
- Lidar publishes at ~7.4 Hz instead of the configured 10 Hz due to
  simulation load — this slightly reduces SLAM update frequency but does
  not affect correctness.
- Maps recorded before the URDF fixes below are invalid and should be
  re-recorded.

## Fixed issues (Lyrical migration)

- **Robot drove backwards / turned the wrong way:** the wheel joints were
  rotated `+pi/2`, so their axis pointed to -Y and the diff drive plugin
  spun them backwards while the odometry reported the opposite motion.
  Fixed with `rpy="-pi/2 0 0"`.
- **Phantom obstacles around the robot:** wheels and caster sank into the
  ground, so the robot rested tilted ~2.3 deg and the Lidar beams hit the
  floor 3-10 m away. `base_footprint` is now raised by
  `wheel_radius - wheel_offset_z` and the caster touches the ground.
- **Odometry under-estimated turns by ~18%:** cylinder wheel collisions
  touched the ground at their inner edge (0.22 m track instead of 0.26 m).
  Wheel collisions are now spheres.
- **slam_toolbox never activated / lifecycle manager errors:** the Nav2
  lifecycle manager could not bond with slam_toolbox, and enabling the bond
  crashed slam_toolbox (`bad_weak_ptr`). The launch file now drives the
  lifecycle transitions itself.
- `loop_match_maximum_variance_covariance` (ignored typo) renamed to
  `loop_match_maximum_variance_coarse`; `min_laser_range` set to 0.12 m.