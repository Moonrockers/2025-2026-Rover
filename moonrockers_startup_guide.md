# Moonrockers Rover — Startup Guide

## What You Need
- PC with PowerShell
- Pi powered on and connected to the same WiFi as your PC
- Pi IP address: `192.168.0.102`
- Pi username: `sdminesmoonrockers`

---

## Every Time You Want to Drive

### Step 1 — Open 3 PowerShell Windows

---

### Window 1 — Start the rover nodes
SSH into the Pi and start the container:
```powershell
ssh sdminesmoonrockers@192.168.0.102
```
Start a fresh container:
```bash
docker run -it --rm --net=host my_ros_image
```
Inside the container, source and run the motor controller:
```bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
ros2 run mars_rover motor_controller
```
✅ You should see:
```
[INFO] Motor controller ready — driver: sparkmax, mode: differential
```

---

### Window 2 — Keyboard control
SSH into the Pi again and open a second shell in the container:
```powershell
ssh sdminesmoonrockers@192.168.0.102
docker exec -it $(docker ps -q) bash
```
Source and run the keyboard teleop:
```bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
✅ You should see the keyboard control instructions printed out.

---

### Window 3 — Monitoring (optional but useful)
SSH into the Pi and open another shell in the container:
```powershell
ssh sdminesmoonrockers@192.168.0.102
docker exec -it $(docker ps -q) bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
```
Useful commands to monitor the rover:
```bash
# Watch motor speeds
ros2 topic echo /motor/left_speed
ros2 topic echo /motor/right_speed

# Check all nodes are running
ros2 node list

# Check all topics
ros2 topic list
```

---

## Keyboard Controls

| Key | Action |
|-----|--------|
| `i` | Forward |
| `,` | Backward |
| `j` | Turn left |
| `l` | Turn right |
| `k` | Stop |
| `u` | Forward + left |
| `o` | Forward + right |
| `q` / `z` | Increase / decrease speed |
| `w` / `x` | Increase / decrease linear speed |
| `e` / `c` | Increase / decrease turn speed |
| `Ctrl+C` | Quit |

---

## Manual ROS2 Drive Commands
If you don't want to use the keyboard teleop, you can send commands directly.
Run these in any container shell (Window 2 or 3):

```bash
# Drive forward
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --rate 10

# Turn left
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}" --rate 10

# Stop
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once

# Emergency stop
ros2 topic pub /emergency_stop std_msgs/msg/Bool "{data: true}" --once

# Release emergency stop
ros2 topic pub /emergency_stop std_msgs/msg/Bool "{data: false}" --once
```

---

## Excavation & Deposition Commands
Run these in any container shell:

```bash
# Start digging
ros2 topic pub /digging/command std_msgs/msg/String "{data: 'start'}" --once

# Stop digging
ros2 topic pub /digging/command std_msgs/msg/String "{data: 'stop'}" --once

# Raise excavator
ros2 topic pub /digging/command std_msgs/msg/String "{data: 'raise'}" --once

# Lower excavator
ros2 topic pub /digging/command std_msgs/msg/String "{data: 'lower'}" --once

# Set dig depth (meters)
ros2 topic pub /digging/target_depth std_msgs/msg/Float32 "{data: 0.1}" --once

# Dump material
ros2 topic pub /deposition/command std_msgs/msg/String "{data: 'dump'}" --once

# Return to neutral
ros2 topic pub /deposition/command std_msgs/msg/String "{data: 'reset'}" --once
```

---

## If the Container Gets Reset
If you restart the Pi or the container stops, the workspace will still be saved
since we committed it to the image. Just start a new container and it will work:
```bash
docker run -it --rm --net=host my_ros_image
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
ros2 run mars_rover motor_controller
```

---

## If You Need to Rebuild
Only needed if you change code files:
```bash
# In Window 3 — copy updated file to Pi
scp $HOME\Downloads\yourfile.py sdminesmoonrockers@192.168.0.102:~/

# Copy into container
docker cp ~/yourfile.py $(docker ps -q):/ros2_ws/src/mars_rover/mars_rover/

# In Window 1 — rebuild
cd /ros2_ws
colcon build
source /ros2_ws/install/setup.bash

# Save the new state permanently
docker commit $(docker ps -q) my_ros_image
```

---

## Quick Troubleshooting

| Problem | Fix |
|---------|-----|
| `Package 'mars_rover' not found` | Run `source /opt/ros/jazzy/setup.bash` then `source /ros2_ws/install/setup.bash` |
| `ssh: Could not resolve hostname` | Use IP directly: `ssh sdminesmoonrockers@192.168.0.102` |
| Motors stop after 1 second | Normal — watchdog timeout. Use `--rate 10` on topic pub commands |
| Container ID changed | Run `docker ps` to get the new ID |
| Everything is gone after restart | Run `docker run -it --rm --net=host my_ros_image` — files are saved in the image |

---

*Team Moonrockers — NASA Lunabotics 2026*
