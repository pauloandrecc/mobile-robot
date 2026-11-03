# robot_description

Contains the URDF/Xacro model of the differential drive robot, the Gazebo
worlds, and the RViz launch file for visualization.

## Package structure

- robot_description/
  - urdf/
    - robot.urdf.xacro   — main robot model (links, joints, inertias)
    - gazebo.xacro       — Gazebo materials, plugins and sensors
  - worlds/
    - empty_world.sdf      — ground plane and sunlight
    - obstacles_world.sdf  — 10×10 m room with boxes and a cylinder
  - launch/
    - display.launch.py  — visualize robot in RViz without simulation
  - rviz/
    - robot.rviz         — robot model and TF
    - slam.rviz          — map, Lidar, robot and TF (used by SLAM)

## Robot model

- Chassis: 30×20×10 cm, 2 kg
- Drive wheels: 2 × continuous joints (left, right), 5 cm radius,
  26 cm apart (sphere collisions, see `robot.urdf.xacro`)
- Caster wheel: fixed joint, front support
- Lidar: fixed joint, top of chassis — 360 rays (1°), 0.12–10 m, 10 Hz
- IMU: fixed joint, center of chassis — 100 Hz, Gaussian noise
- Camera: fixed joint, front face of chassis (below the Lidar scan plane)
  — 640×480 RGB, 60° horizontal FOV, 15 Hz

## TF tree

```
map                      (published by slam_toolbox, when running)
└── odom                 (published by the diff drive plugin)
    └── base_footprint
        └── base_link
            ├── left_wheel
            ├── right_wheel
            ├── caster_wheel
            ├── lidar_link
            ├── imu_link
            └── camera_link
                └── camera_optical_link
```

`camera_optical_link` follows the ROS optical frame convention (REP 103):
z forward, x right, y down. Images and camera info are expressed in it,
while `camera_link` keeps the robot convention (x forward, z up).

## Gazebo plugins and sensors

| Plugin / sensor | Purpose |
|-----------------|---------|
| `gz-sim-diff-drive-system` | Subscribes to `/cmd_vel`, publishes `/odom` and `/tf` |
| `gz-sim-joint-state-publisher-system` | Publishes wheel joint positions to `/joint_states` |
| `gpu_lidar` sensor | Publishes laser scan data to `/scan` |
| `imu` sensor | Publishes angular velocity and linear acceleration to `/imu` |
| `camera` sensor | Publishes images to `/camera/image_raw` and `/camera/camera_info` |

The worlds must load the matching *systems*: `gz-sim-sensors-system`
renders the Lidar and the camera, and `gz-sim-imu-system` computes the IMU
data. A sensor whose system is missing exists in the model but never
publishes.

## ROS 2 topics

| Topic | Type | Direction |
|-------|------|-----------|
| `/cmd_vel` | `geometry_msgs/Twist` | input — velocity commands |
| `/odom` | `nav_msgs/Odometry` | output — wheel odometry |
| `/scan` | `sensor_msgs/LaserScan` | output — Lidar data |
| `/imu` | `sensor_msgs/Imu` | output — IMU data |
| `/camera/image_raw` | `sensor_msgs/Image` | output — camera image (rgb8) |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | output — camera calibration |
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
- Sensor rates depend on the machine load. Measured with headless rendering
  (real-time factor ~1.0): `/scan` ~10 Hz, `/imu` 100 Hz, `/odom` 20 Hz,
  `/camera/image_raw` ~13 Hz (configured 15 Hz).