# Mobile Robot — ROS 2

![License](https://img.shields.io/badge/license-Apache%202.0-blue.svg)
![ROS 2](https://img.shields.io/badge/ROS%202-Kilted-blue)
![Status](https://img.shields.io/badge/status-em%20desenvolvimento-yellow)

A differential drive mobile robot built with ROS 2, simulated in Gazebo
with autonomous navigation using Nav2 and SLAM.

## Project structure

- mobile_robot/
- src/
- robot_description/   — URDF model, RViz config, display launch
- robot_bringup/       — top-level launch files and config

## Requirements

- ROS 2 Kilted
- Gazebo Harmonic: `sudo apt install ros-kilted-ros-gz`
- Xacro: `sudo apt install ros-kilted-xacro`
- Joint state publisher GUI: `sudo apt install ros-kilted-joint-state-publisher-gui`
- rqt graph: `sudo apt install ros-kilted-rqt-graph`

Or install all dependencies at once:

```bash
cd ~/mobile_robot
rosdep install --from-paths src --ignore-src -r -y
```

## Build

```bash
cd ~/mobile_robot
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```
## Usage

### Visualize the robot model (no simulation)

```bash
ros2 launch robot_description display.launch.py
```

### Run the full Gazebo simulation

```bash
ros2 launch robot_bringup simulation.launch.py
```

> **Important:** After Gazebo opens, click the **play button** in the
> bottom-left corner to start the simulation clock. The robot will not
> respond to commands until the simulation is running.

### Teleoperate the robot

In a separate terminal after launching the simulation:

```bash
# Move forward
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.0}}" --rate 10

# Rotate
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.5}}" --rate 10

# Stop
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}" --once
```

### Inspect the system

```bash
# List all active topics
ros2 topic list

# Check publishing rates
ros2 topic hz /scan
ros2 topic hz /odom

# View the TF tree
ros2 run tf2_tools view_frames

# View the node graph
ros2 run rqt_graph rqt_graph
```

## Phases

- [x] Phase 1 — Robot model (URDF, TF2, RViz)
- [x] Phase 2 — Gazebo simulation (diff drive, Lidar, odometry)
- [ ] Phase 3 — SLAM and mapping
- [ ] Phase 4 — Custom interfaces and action server
- [ ] Phase 5 — Full autonomous navigation with Nav2
- [ ] Phase 6 — Tests

## License

This project is licensed under the Apache License 2.0 —
see the [LICENSE](LICENSE) file for details.