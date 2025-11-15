# Getting Started with the Rover

This guide will help you get started with the rover control system.

## Prerequisites

Before you begin, ensure you have:

1. **ROS2 Installed**: 
   - Recommended: ROS2 Humble or later
   - Installation guide: https://docs.ros.org/en/humble/Installation.html

2. **Python 3.8+**:
   ```bash
   python3 --version
   ```

3. **Colcon Build Tool**:
   ```bash
   sudo apt install python3-colcon-common-extensions
   ```

4. **ROS2 Dependencies**:
   ```bash
   sudo apt install ros-humble-geometry-msgs ros-humble-sensor-msgs ros-humble-nav-msgs
   ```

## Quick Start

### 1. Create a ROS2 Workspace

```bash
mkdir -p ~/rover_ws/src
cd ~/rover_ws/src
```

### 2. Clone the Repository

```bash
git clone <repository-url> 2025-2026-Rover
```

### 3. Build the Package

```bash
cd ~/rover_ws
colcon build --packages-select rover_control
```

### 4. Source the Workspace

```bash
source ~/rover_ws/install/setup.bash
```

Add this to your `.bashrc` for convenience:
```bash
echo "source ~/rover_ws/install/setup.bash" >> ~/.bashrc
```

### 5. Launch the Rover System

```bash
ros2 launch rover_control rover_bringup.launch.py
```

### 6. Start Teleoperation (in new terminal)

```bash
source ~/rover_ws/install/setup.bash
ros2 run rover_control teleop_keyboard
```

## Verification

### Check Running Nodes

```bash
ros2 node list
```

You should see:
- `/motor_controller`
- `/sensor_manager`
- `/telemetry`
- `/safety_monitor`
- `/teleop_keyboard` (if running)

### Check Active Topics

```bash
ros2 topic list
```

### Monitor a Topic

```bash
ros2 topic echo /cmd_vel
ros2 topic echo /telemetry
```

### Visualize Topics

```bash
rqt_graph
```

## Hardware Setup

### Connecting to the Rover

1. **Power on the rover**
2. **Connect via SSH** (if using remote computer):
   ```bash
   ssh rover@<rover-ip-address>
   ```

3. **Verify hardware connections**:
   - Motors connected to motor controller
   - Sensors properly wired
   - Camera(s) detected
   - IMU/GPS modules connected

### Testing Motors

Test individual motors without full system:
```bash
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}" --once
```

### Testing Sensors

View camera feed:
```bash
ros2 run image_tools showimage --ros-args -r image:=/camera/front/image_raw
```

Check IMU data:
```bash
ros2 topic echo /imu/data
```

## Troubleshooting

### Package Not Found
- Make sure you sourced the workspace: `source ~/rover_ws/install/setup.bash`
- Rebuild: `colcon build --packages-select rover_control`

### No Topics Appearing
- Check if nodes are running: `ros2 node list`
- Check for errors in terminal output

### Motors Not Responding
- Verify hardware connections
- Check emergency stop status: `ros2 topic echo /emergency_stop`
- Review motor controller logs

### Permission Issues
- Add user to dialout group for serial devices:
  ```bash
  sudo usermod -aG dialout $USER
  ```
- Log out and back in for changes to take effect

## Next Steps

- Read the [Architecture Documentation](architecture.md)
- Explore the [Configuration Guide](configuration.md)
- Set up autonomous navigation
- Integrate additional sensors

## Getting Help

- Check the README.md for detailed information
- Review the example code in last year's repository: robo52/moonrockers
- Open an issue on GitHub for bugs or questions
