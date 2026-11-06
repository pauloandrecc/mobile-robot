# Mobile Robot — ROS 2

![License](https://img.shields.io/badge/license-Apache%202.0-blue.svg)
![ROS 2](https://img.shields.io/badge/ROS%202-Lyrical-blue)
![Status](https://img.shields.io/badge/status-em%20desenvolvimento-yellow)

A differential drive mobile robot built with ROS 2, simulated in Gazebo
with autonomous navigation using Nav2 and SLAM.

## Project structure

```
mobile-robot/
├── src/
│   ├── robot_description/   — URDF model (Lidar, IMU, camera), worlds, RViz configs
│   ├── robot_bringup/       — top-level launch files, config, helper nodes
│   ├── robot_interfaces/    — custom action and service definitions
│   └── robot_navigation/    — waypoint action server and client (Python)
├── maps/                    — saved maps from SLAM sessions
├── bags/                    — recorded bags (not versioned)
└── AGENTS.md                — roadmap, phase status and conventions
```

## Requirements

- ROS 2 Lyrical Luth (Ubuntu 26.04)
- Gazebo Jetty: `sudo apt install ros-lyrical-ros-gz`
- Xacro: `sudo apt install ros-lyrical-xacro`
- Joint state publisher GUI: `sudo apt install ros-lyrical-joint-state-publisher-gui`
- rqt graph: `sudo apt install ros-lyrical-rqt-graph`
- SLAM Toolbox: `sudo apt install ros-lyrical-slam-toolbox`
- Nav2: `sudo apt install ros-lyrical-nav2-bringup`
- Teleop: `sudo apt install ros-lyrical-teleop-twist-keyboard`
- Gamepad: `sudo apt install ros-lyrical-joy ros-lyrical-teleop-twist-joy`

Or install all dependencies at once (from the repository root):

```bash
rosdep install --from-paths src --ignore-src -r -y
```

## Build

```bash
git clone https://github.com/pauloandrecc/mobile-robot.git
cd mobile-robot
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

> **zsh users:** source the `.zsh` files instead (`source /opt/ros/lyrical/setup.zsh`
> and `source install/setup.zsh`). Sourcing the `.bash` scripts from zsh fails
> with `not found: ".../local_setup.bash"`.

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

### Run SLAM mapping

```bash
ros2 launch robot_bringup slam.launch.py
```

In a separate terminal, teleoperate the robot to explore the environment:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

After exploring, save the map (from the repository root):

```bash
ros2 run nav2_map_server map_saver_cli -f maps/my_map
```

### Record a mapping session and replay it without Gazebo

```bash
# Record /scan, /odom, /tf, /imu, ... to bags/mapping_<date> (MCAP)
ros2 launch robot_bringup slam.launch.py record:=true

# Inspect the bag
ros2 bag info bags/mapping_<date>

# Rebuild the map from the bag only — no Gazebo running
ros2 launch robot_bringup slam_replay.launch.py bag:=bags/mapping_<date>
```

### Navigate through waypoints (Nav2)

Starts Gazebo, slam_toolbox, a minimal Nav2 stack and the waypoint action
server (`/navigate_to_waypoints`):

```bash
ros2 launch robot_bringup navigation.launch.py
```

When the log shows `Managed nodes are active`, send waypoints (`x,y` or
`x,y,yaw_deg`, map frame — the map origin is where the robot started):

```bash
ros2 run robot_navigation navigation_client 2.5,0 3.2,2.0,90 0,0,180

# Per-goal speed limit (m/s); use -- before negative coordinates
ros2 run robot_navigation navigation_client --max-speed 0.15 -- -3,0.5
```

The client prints the feedback once per second; **Ctrl+C cancels** the goal
and the robot stops. Change the speed limit while it drives:

```bash
ros2 service call /set_speed_limit robot_interfaces/srv/SetSpeedLimit "{max_speed: 0.1}"
# same effect, through the parameter directly
ros2 param set /waypoint_action_server max_speed 0.1
```

Or use the action from the CLI:

```bash
ros2 action send_goal --feedback /navigate_to_waypoints \
  robot_interfaces/action/NavigateToWaypoints \
  "{waypoints: [{header: {frame_id: map}, pose: {position: {x: 1.5}, orientation: {w: 1.0}}}]}"
```

### Teleoperate with a gamepad

The simulation (and SLAM) launch starts gamepad teleoperation by default
(`joy` + `teleop_twist_joy`). Tested with a GameSir Nova Lite and its
2.4 GHz receiver (seen by Linux as "Zikway HID gamepad"). The controller is
read through SDL's GameController mapping, so any controller SDL knows
(Xbox, PlayStation, ...) uses the same buttons.

| Control | Action |
|---------|--------|
| Hold **LB** | Enable driving (release to stop) |
| Hold **RB** | Enable driving in turbo mode |
| Left stick ↑/↓ | Forward / backward (0.3 m/s, turbo 0.6 m/s) |
| Left stick ←/→ | Turn (1.0 rad/s, turbo 1.5 rad/s) |

Mappings and speeds live in `src/robot_bringup/config/joystick.yaml`.
To disable it: `ros2 launch robot_bringup simulation.launch.py use_joystick:=false`.
To check the controller indices: `ros2 topic echo /joy`.

### Teleoperate the robot from the terminal

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

# Check publishing rates (expected: /scan ~10 Hz, /imu 100 Hz,
# /odom 20 Hz, /camera/image_raw ~13-15 Hz)
ros2 topic hz /scan
ros2 topic hz /imu
ros2 topic hz /odom
ros2 topic hz /camera/image_raw

# Inspect topics, rates and message contents in a GUI
ros2 run rqt_topic rqt_topic

# See the camera image
ros2 run rqt_image_view rqt_image_view /camera/image_raw

# Plot the IMU gyroscope while turning the robot
ros2 run rqt_plot rqt_plot /imu/angular_velocity/z

# View the TF tree
ros2 run tf2_tools view_frames

# View the node graph
ros2 run rqt_graph rqt_graph
```

## Phases

- [x] Phase 1 — Robot model (URDF, TF2, RViz)
- [x] Phase 2 — Gazebo simulation (diff drive, Lidar, IMU, camera, odometry)
- [x] Phase 3 — SLAM mapping (slam_toolbox, map saving, bag recording)
- [x] Phase 4 — Custom interfaces and action server
- [ ] Phase 5 — Full autonomous navigation with Nav2
- [ ] Phase 6 — Tests

## License

This project is licensed under the Apache License 2.0 —
see the [LICENSE](LICENSE) file for details.