# Mobile Robot — ROS 2

A differential drive mobile robot built with ROS 2, simulated in Gazebo
with autonomous navigation using Nav2 and SLAM.

## Project structure

- mobile_robot/
- src/
- robot_description/   — URDF model, RViz config, display launch
- robot_bringup/       — top-level launch files and config

## Requirements

- ROS 2 Kilted
- Xacro: `sudo apt install ros-kilted-xacro`
- Joint state publisher GUI: `sudo apt install ros-kilted-joint-state-publisher-gui`

## Build

```bash
cd ~/mobile_robot
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Phase 1 — Robot model and visualization

Launches the URDF model in RViz with the joint state publisher GUI:

```bash
ros2 launch robot_description display.launch.py
```

## Phases

- [x] Phase 1 — Robot model (URDF, TF2, RViz)
- [ ] Phase 2 — Gazebo simulation
- [ ] Phase 3 — SLAM and mapping
- [ ] Phase 4 — Custom interfaces and action server
- [ ] Phase 5 — Full autonomous navigation with Nav2
- [ ] Phase 6 — Tests