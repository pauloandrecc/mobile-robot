# robot_description

Contains the URDF/Xacro model of the differential drive robot and
the RViz launch file for visualization.

## Package structure

- robot_description/
- urdf/
- robot.urdf.xacro   — main robot model (links, joints, inertias)
- gazebo.xacro       — Gazebo material colors and simulation plugins
- worlds/
- empty_world.sdf    — Gazebo world with ground plane and sunlight
- launch/
- display.launch.py  — visualize robot in RViz without simulation
- rviz/
- my_robot.rviz      — RViz configuration

## Robot model

- Chassis: 30×20×10 cm, 2 kg
- Drive wheels: 2 × continuous joints (left, right)
- Caster wheel: fixed joint, front support
- Lidar: fixed joint, top of chassis

## TF tree

```
odom
└── base_footprint
└── base_link
├── left_wheel
├── right_wheel
├── caster_wheel
└── lidar_link
base_footprint
  └── base_link
        ├── left_wheel
        ├── right_wheel
        ├── caster_wheel
        └── lidar_link
```

## Gazebo plugins

| Plugin | Purpose |
|--------|---------|
| `gz-sim-diff-drive-system` | Subscribes to `/cmd_vel`, publishes `/odom` and `/tf` |
| `gz-sim-joint-state-publisher-system` | Publishes wheel joint positions to `/joint_states` |
| `gpu_lidar` sensor | Publishes laser scan data to `/scan` |

## ROS 2 topics

| Topic | Type | Direction |
|-------|------|-----------|
| `/cmd_vel` | `geometry_msgs/Twist` | input — velocity commands |
| `/odom` | `nav_msgs/Odometry` | output — wheel odometry |
| `/scan` | `sensor_msgs/LaserScan` | output — Lidar data |
| `/joint_states` | `sensor_msgs/JointState` | output — wheel joint positions |
| `/tf` | `tf2_msgs/TFMessage` | output — odom→base_footprint transform |
| `/clock` | `rosgraph_msgs/Clock` | output — simulation time |

## Launch

```bash
ros2 launch robot_description display.launch.py
# Disable joint_state_publisher_gui:
ros2 launch robot_description display.launch.py use_gui:=false
```

## Known issues

- The Gazebo simulation must be manually started by clicking the
  **play button** after launch — the clock only begins after play is pressed,
  and a stopped clock causes `/joint_states` timestamps to be zero, which
  makes `robot_state_publisher` reject the transforms.
- Lidar publishes at ~7.4 Hz instead of the configured 10 Hz due to
  simulation load — this is normal behavior under CPU load.