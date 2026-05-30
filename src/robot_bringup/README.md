# robot_bringup

Top-level launch files and configuration for the mobile robot.
This package orchestrates all other packages — simulation, SLAM, and navigation.

## Package structure

- robot_bringup/
- launch/
- simulation.launch.py      — Gazebo simulation with diff drive and Lidar
- slam.launch.py            — SLAM mapping with slam_toolbox
- config/
- slam_toolbox_params.yaml  — slam_toolbox tuning parameters

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
| `world` | `obstacles_world.sdf` | Path to the Gazebo world file |
| `world_name` | `obstacles_world` | World name (used for joint state topic bridge) |

Example — launch with a custom world:

```bash
ros2 launch robot_bringup simulation.launch.py \
  world:=/path/to/world.sdf \
  world_name:=my_world
```

### slam.launch.py

Launches the full simulation and starts SLAM mapping with slam_toolbox.
Includes a lifecycle manager to activate the slam_toolbox node automatically.

```bash
ros2 launch robot_bringup slam.launch.py
```

Arguments:

| Argument | Default | Description |
|----------|---------|-------------|
| `use_sim_time` | `true` | Use simulation clock |

> **Important:** After launching, click the **play button** in Gazebo and
> wait ~6 seconds before moving the robot. This gives the SLAM node time
> to initialize and all TF frames time to stabilize.

## SLAM workflow

1. Launch SLAM: `ros2 launch robot_bringup slam.launch.py`
2. Click play in Gazebo and wait 6 seconds
3. Teleoperate: `ros2 run teleop_twist_keyboard teleop_twist_keyboard`
4. Explore the environment slowly — prefer straight lines over sharp turns
5. Save the map: `ros2 run nav2_map_server map_saver_cli -f ~/mobile_robot/maps/my_map`

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

- Map quality degrades with sharp turns due to odometry drift — move
  slowly and prefer gradual rotations for best results.
- The slam_toolbox node must be started after the simulation clock is
  running — the 6-second delay in the launch file handles this automatically.
- Lidar publishes at ~7.4 Hz instead of the configured 10 Hz due to
  simulation load — this slightly reduces SLAM update frequency but does
  not affect correctness.
- Map quality will improve in Phase 5 when AMCL localization is added
  on top of the saved map.