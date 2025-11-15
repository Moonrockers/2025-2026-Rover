# 2025-2026-Rover

ROS2-based control system for the Moonrockers rover robot.

## Overview

This repository contains a ROS2 package for controlling a rover robot with teleoperation capabilities. The system includes motor control, sensor management, telemetry, and safety monitoring.

## Reference

If you search up the repository "robo52/moonrockers" you will find last year's code for reference.

## Prerequisites

- ROS2 (Humble or later recommended)
- Python 3.8+
- colcon build tool

## Package Structure

```
rover_control/
├── rover_control/           # Main Python package
│   ├── motor_controller.py  # Motor control node
│   ├── teleop_keyboard.py   # Keyboard teleoperation
│   ├── sensor_manager.py    # Sensor data management
│   ├── telemetry.py         # Telemetry aggregation
│   └── safety_monitor.py    # Safety monitoring
├── launch/                  # Launch files
│   ├── rover_bringup.launch.py  # Main system launch
│   └── teleop.launch.py         # Teleoperation launch
├── config/                  # Configuration files
│   └── rover_params.yaml    # Parameter configuration
├── package.xml              # Package metadata
└── setup.py                 # Python package setup
```

## Installation

1. Clone this repository into your ROS2 workspace:
```bash
cd ~/ros2_ws/src
git clone <repository-url> 2025-2026-Rover
```

2. Build the workspace:
```bash
cd ~/ros2_ws
colcon build --packages-select rover_control
```

3. Source the workspace:
```bash
source ~/ros2_ws/install/setup.bash
```

## Usage

### Launch the main rover system:
```bash
ros2 launch rover_control rover_bringup.launch.py
```

This will start:
- Motor controller
- Sensor manager
- Telemetry node
- Safety monitor

### Launch keyboard teleoperation (in a separate terminal):
```bash
ros2 launch rover_control teleop.launch.py
```

Or run the teleop node directly:
```bash
ros2 run rover_control teleop_keyboard
```

### Control the rover:
- **w/s**: Move forward/backward
- **a/d**: Turn left/right
- **x**: Stop
- **q/z**: Increase/decrease speed
- **Ctrl+C**: Quit

## Topics

### Published Topics
- `/cmd_vel` - Velocity commands (geometry_msgs/Twist)
- `/motor_speeds` - Individual motor speeds (std_msgs/Float64MultiArray)
- `/camera/front/image_raw` - Front camera feed (sensor_msgs/Image)
- `/imu/data` - IMU data (sensor_msgs/Imu)
- `/gps/fix` - GPS position (sensor_msgs/NavSatFix)
- `/telemetry` - Consolidated telemetry data (std_msgs/String)
- `/emergency_stop` - Emergency stop signal (std_msgs/Bool)

### Subscribed Topics
- `/cmd_vel` - Velocity commands for motor control
- `/emergency_stop` - Emergency stop commands

## Configuration

Edit `rover_control/config/rover_params.yaml` to adjust:
- Wheel base and radius
- Maximum speeds
- Safety thresholds
- Sensor publish rates

## Development

### Adding New Nodes

1. Create a new Python file in `rover_control/rover_control/`
2. Add the entry point in `setup.py`
3. Add the node to the appropriate launch file

### Testing

Run tests with:
```bash
colcon test --packages-select rover_control
colcon test-result --verbose
```

## Safety Features

- **Emergency Stop**: Monitors battery voltage and temperature
- **Command Timeout**: Motors stop if no commands received for 1 second
- **Speed Limiting**: Enforces maximum speed limits

## License

Apache-2.0

## Team

Moonrockers Team - team@moonrockers.com
