# robot_description

Contains the URDF/Xacro model of the differential drive robot and
the RViz launch file for visualization.

## Robot model

- Chassis: 30×20×10 cm, 2 kg
- Drive wheels: 2 × continuous joints (left, right)
- Caster wheel: fixed joint, front support
- Lidar: fixed joint, top of chassis

## Launch

```bash
ros2 launch robot_description display.launch.py
# Disable joint_state_publisher_gui:
ros2 launch robot_description display.launch.py use_gui:=false
```

## TF tree

```
base_footprint
  └── base_link
        ├── left_wheel
        ├── right_wheel
        ├── caster_wheel
        └── lidar_link
```