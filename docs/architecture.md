# Rover System Architecture

## Overview

The rover control system is built using ROS2 and follows a modular architecture with separate nodes for different functionalities.

## System Diagram

```
┌─────────────────┐
│  Teleop Input   │
│  (Keyboard/Joy) │
└────────┬────────┘
         │
         v
    ┌────────┐
    │cmd_vel │ topic
    └────┬───┘
         │
         v
┌────────────────────┐     ┌──────────────┐
│ Motor Controller   │────>│ Motor Speeds │
└────────────────────┘     └──────────────┘
         ^
         │
    ┌────┴─────┐
    │Emergency │
    │   Stop   │
    └──────────┘
         ^
         │
┌────────────────────┐
│ Safety Monitor     │
└────────────────────┘
         ^
         │
┌────────────────────┐
│ Sensor Manager     │
└────────────────────┘
         │
         v
    ┌────────┐
    │Various │
    │Sensors │
    └────┬───┘
         │
         v
┌────────────────────┐
│   Telemetry        │
└────────────────────┘
```

## Nodes

### Motor Controller
- **Purpose**: Converts velocity commands to individual motor speeds
- **Subscribes to**: `/cmd_vel`, `/emergency_stop`
- **Publishes to**: `/motor_speeds`
- **Key Features**: 
  - Differential drive kinematics
  - Speed limiting
  - Watchdog timer for safety

### Teleop Keyboard
- **Purpose**: Manual control via keyboard
- **Publishes to**: `/cmd_vel`
- **Key Features**:
  - WASD control
  - Variable speed control
  - Emergency stop capability

### Sensor Manager
- **Purpose**: Interface with all rover sensors
- **Publishes to**: 
  - `/camera/front/image_raw`
  - `/camera/rear/image_raw`
  - `/imu/data`
  - `/gps/fix`
  - `/temperature`
  - `/battery_voltage`
  - `/range/front`, `/range/rear`
- **Key Features**:
  - Multi-sensor integration
  - Configurable publish rates
  - Hardware abstraction

### Telemetry
- **Purpose**: Aggregate and transmit rover status
- **Subscribes to**: All sensor topics
- **Publishes to**: `/telemetry`
- **Key Features**:
  - JSON formatted data
  - Configurable transmission rate
  - Ground station communication ready

### Safety Monitor
- **Purpose**: Monitor critical parameters
- **Subscribes to**: Battery, temperature, IMU topics
- **Publishes to**: `/emergency_stop`
- **Key Features**:
  - Battery level monitoring
  - Temperature monitoring
  - Tilt angle monitoring
  - Automatic emergency stop

## Communication

### ROS2 Topics
All inter-node communication uses ROS2 topics with standard message types where possible.

### Message Types
- `geometry_msgs/Twist` - Velocity commands
- `sensor_msgs/Image` - Camera data
- `sensor_msgs/Imu` - IMU data
- `sensor_msgs/NavSatFix` - GPS data
- `std_msgs/Float64MultiArray` - Motor speeds
- `std_msgs/Bool` - Emergency stop

## Configuration

Parameters are loaded from YAML files in the `config/` directory. Each node can be configured independently.

## Safety Features

1. **Emergency Stop**: Can be triggered manually or automatically
2. **Command Timeout**: Motors stop if no commands for 1 second
3. **Parameter Monitoring**: Battery, temperature, and tilt monitoring
4. **Speed Limiting**: Enforced maximum speeds

## Future Enhancements

- [ ] Add autonomous navigation capabilities
- [ ] Implement obstacle detection and avoidance
- [ ] Add camera-based vision processing
- [ ] Implement RealSense depth camera integration
- [ ] Add GPS waypoint navigation
- [ ] Implement SLAM for mapping
