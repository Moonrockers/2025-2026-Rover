# Rover Description

This directory contains the robot description files (URDF/XACRO) for the rover.

## Files

- `urdf/rover.urdf.xacro` - Main robot description file
- `meshes/` - 3D mesh files for visualization (STL/DAE files)

## Usage

### View the robot model in RViz:

```bash
ros2 launch urdf_tutorial display.launch.py model:=/path/to/rover.urdf.xacro
```

### Generate URDF from XACRO:

```bash
xacro rover_description/urdf/rover.urdf.xacro > rover.urdf
```

## Robot Components

The rover model includes:
- Base link (main chassis)
- 4 wheels (differential drive)
- IMU sensor
- GPS sensor
- Camera

## Coordinate Frames

- `base_footprint` - Ground projection of the robot center
- `base_link` - Main robot body
- `imu_link` - IMU sensor frame
- `gps_link` - GPS sensor frame
- `camera_link` - Camera frame
- `*_wheel` - Individual wheel frames
