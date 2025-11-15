# Quick Reference Guide

## Common Commands

### Building
```bash
cd ~/ros2_ws
colcon build --packages-select rover_control
source install/setup.bash
```

### Running

**Start all core nodes:**
```bash
ros2 launch rover_control rover_bringup.launch.py
```

**Start teleoperation:**
```bash
ros2 run rover_control teleop_keyboard
```

**Start individual nodes:**
```bash
ros2 run rover_control motor_controller
ros2 run rover_control sensor_manager
ros2 run rover_control telemetry
ros2 run rover_control safety_monitor
```

### Monitoring

**List active nodes:**
```bash
ros2 node list
```

**List topics:**
```bash
ros2 topic list
```

**Echo a topic:**
```bash
ros2 topic echo /cmd_vel
ros2 topic echo /motor_speeds
ros2 topic echo /telemetry
```

**View topic info:**
```bash
ros2 topic info /cmd_vel
```

**View node graph:**
```bash
rqt_graph
```

### Testing

**Publish test velocity command:**
```bash
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}" --once
```

**Trigger emergency stop:**
```bash
ros2 topic pub /emergency_stop std_msgs/Bool "data: true" --once
```

**Release emergency stop:**
```bash
ros2 topic pub /emergency_stop std_msgs/Bool "data: false" --once
```

### Parameters

**List parameters for a node:**
```bash
ros2 param list /motor_controller
```

**Get parameter value:**
```bash
ros2 param get /motor_controller wheel_base
```

**Set parameter value:**
```bash
ros2 param set /motor_controller max_speed 1.5
```

## Keyboard Controls

When running `teleop_keyboard`:

| Key | Action |
|-----|--------|
| w   | Forward |
| s   | Backward |
| a   | Turn left |
| d   | Turn right |
| x   | Stop |
| q   | Increase speed |
| z   | Decrease speed |
| Ctrl+C | Quit |

## Important Topics

| Topic | Message Type | Description |
|-------|-------------|-------------|
| /cmd_vel | geometry_msgs/Twist | Velocity commands |
| /motor_speeds | std_msgs/Float64MultiArray | Motor speeds |
| /emergency_stop | std_msgs/Bool | Emergency stop signal |
| /imu/data | sensor_msgs/Imu | IMU data |
| /gps/fix | sensor_msgs/NavSatFix | GPS position |
| /telemetry | std_msgs/String | Consolidated telemetry |
| /battery_voltage | std_msgs/Float32 | Battery voltage |
| /temperature | sensor_msgs/Temperature | Temperature |

## Configuration Files

- **Parameters**: `rover_control/config/rover_params.yaml`
- **Robot Model**: `rover_description/urdf/rover.urdf.xacro`

## Troubleshooting

**Nodes not starting:**
- Check if ROS2 is sourced: `source /opt/ros/humble/setup.bash`
- Source workspace: `source ~/ros2_ws/install/setup.bash`
- Rebuild: `colcon build --packages-select rover_control`

**Topics not appearing:**
- Check nodes are running: `ros2 node list`
- Check for errors in terminal output

**Motors not responding:**
- Check emergency stop: `ros2 topic echo /emergency_stop`
- Verify hardware connections

## File Locations

- **Main package**: `rover_control/`
- **Python nodes**: `rover_control/rover_control/`
- **Launch files**: `rover_control/launch/`
- **Config files**: `rover_control/config/`
- **Robot description**: `rover_description/urdf/`
- **Documentation**: `docs/`
