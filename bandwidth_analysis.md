# Bandwidth Analysis: Custom SLAM for Rover

## Quick Answer
**Total Bandwidth Required: ~25-35 Mbps (3-4 MB/s)**

This assumes you're streaming all data to/from the rover. However, there are ways to reduce this dramatically!

---

## Detailed Bandwidth Breakdown

### Data FROM Rover (Rover → Base Station)

#### Camera Data:
1. **RGB Image** (640x480 @ 30fps)
   - Resolution: 640 × 480 pixels
   - Color depth: 3 bytes per pixel (BGR)
   - Uncompressed: 640 × 480 × 3 = 921,600 bytes/frame
   - Frame rate: 30 fps
   - **Bandwidth: ~26.4 MB/s = 211 Mbps** 😱

2. **Depth Image** (640x480 @ 30fps)
   - Resolution: 640 × 480 pixels
   - Depth: 2 bytes per pixel (uint16)
   - Uncompressed: 640 × 480 × 2 = 614,400 bytes/frame
   - Frame rate: 30 fps
   - **Bandwidth: ~17.6 MB/s = 141 Mbps** 😱

3. **IMU Data** (200 Hz gyro + 100 Hz accel)
   - Gyro: 3 floats × 4 bytes × 200 Hz = 2.4 KB/s
   - Accel: 3 floats × 4 bytes × 100 Hz = 1.2 KB/s
   - **Bandwidth: ~3.6 KB/s = 0.029 Mbps** ✓

4. **Camera Info** (published once at startup)
   - ~1 KB total
   - **Bandwidth: negligible** ✓

**TOTAL RAW (uncompressed): ~352 Mbps** 😱😱😱

### Data TO Rover (Base Station → Rover)

Assuming you're doing SLAM processing on base station and sending commands back:

1. **Motor Commands**
   - Twist messages: ~48 bytes @ 10 Hz
   - **Bandwidth: ~480 bytes/s = 0.004 Mbps** ✓

2. **Navigation Goals**
   - Occasional pose goals: ~100 bytes @ 1 Hz
   - **Bandwidth: ~100 bytes/s = 0.001 Mbps** ✓

**TOTAL TO ROVER: ~0.005 Mbps** ✓ (basically nothing)

---

## The Problem: 352 Mbps is Way Too Much!

Most rover communication links:
- **WiFi (long range)**: 20-50 Mbps realistic
- **4G/LTE**: 5-20 Mbps
- **LoRa**: 0.3-50 Kbps
- **Radio modems**: 0.1-1 Mbps

**You need compression or on-board processing!**

---

## Solution 1: ON-BOARD SLAM (RECOMMENDED) ⭐

### Run SLAM on the Rover Itself

**Hardware needed:**
- Jetson Nano/Orin Nano: $99-$199
- Raspberry Pi 5: $80
- Intel NUC or similar: $200-500

**Bandwidth FROM rover:**
```
1. Odometry (10 Hz):          ~1 KB/s   = 0.008 Mbps
2. Map updates (1 Hz):        ~160 KB/s = 1.3 Mbps
3. Trajectory (1 Hz):         ~10 KB/s  = 0.08 Mbps
4. Compressed video (10fps):  ~500 KB/s = 4 Mbps (optional monitoring)
────────────────────────────────────────────────
TOTAL:                        ~670 KB/s = 5.4 Mbps ✓✓✓
```

**Bandwidth TO rover:**
```
1. Navigation commands:       ~1 KB/s   = 0.008 Mbps
────────────────────────────────────────────────
TOTAL:                        ~1 KB/s   = 0.008 Mbps ✓✓✓
```

**Total bidirectional: ~5.4 Mbps** ✓ Totally doable on WiFi or 4G!

### Pros:
- ✅ Massively reduced bandwidth (60× reduction!)
- ✅ Lower latency (no round-trip delay)
- ✅ Rover can operate autonomously if connection lost
- ✅ Only send high-level data

### Cons:
- ❌ Need computing hardware on rover
- ❌ Power consumption increases
- ❌ Slight added complexity

---

## Solution 2: IMAGE COMPRESSION

### If You Must Stream Raw Images

Use image compression in your ROS2 pipeline:

```python
# Install image transport plugins
sudo apt install ros-humble-image-transport-plugins

# Launch with compression
ros2 run image_transport republish compressed raw \
  --ros-args -r in/compressed:=/camera/camera/color/image_raw/compressed \
             -r out:=/camera/camera/color/image_raw
```

**Bandwidth with JPEG compression:**

1. **RGB (JPEG, quality=80)**
   - ~30-50 KB per frame @ 30fps
   - **Bandwidth: ~1.2 MB/s = 9.6 Mbps** ✓

2. **Depth (PNG compression)**
   - ~100-150 KB per frame @ 30fps
   - **Bandwidth: ~4 MB/s = 32 Mbps** ⚠️

3. **IMU**: 0.029 Mbps ✓

**TOTAL COMPRESSED: ~42 Mbps** ⚠️ Still high, but more manageable

### Pros:
- ✅ No extra hardware needed
- ✅ Simple to implement

### Cons:
- ❌ Still ~40 Mbps (needs good WiFi)
- ❌ Compression artifacts
- ❌ CPU overhead for compression
- ❌ Latency from compression/decompression

---

## Solution 3: REDUCE FRAME RATE

### Drop to 5-10 fps Instead of 30 fps

SLAM doesn't need 30fps if you're moving slowly!

```python
# In your launch file, reduce camera FPS
'depth_module.profile': '640x480x5',   # Was 30
'rgb_camera.profile': '640x480x5',     # Was 30
```

**Bandwidth at 5 fps (compressed):**
- RGB: ~320 KB/s = 2.6 Mbps ✓
- Depth: ~667 KB/s = 5.3 Mbps ✓
- IMU: 0.029 Mbps ✓

**TOTAL: ~8 Mbps** ✓✓ Works on decent WiFi!

### Pros:
- ✅ Much lower bandwidth
- ✅ No extra hardware
- ✅ Simple configuration change

### Cons:
- ❌ Less smooth tracking (but usually fine for rovers)
- ❌ May lose features if moving fast
- ❌ Still need compression

---

## Solution 4: HYBRID APPROACH (BEST FOR MOST CASES) ⭐⭐⭐

### Run SLAM On-Board + Send Compressed Video for Monitoring

```
┌─────────────────┐         ┌──────────────────┐
│  Rover (Jetson) │         │  Base Station    │
│                 │         │                  │
│  RealSense D435i│         │                  │
│       ↓         │         │                  │
│   SLAM Node     │         │                  │
│       ↓         │         │                  │
│  Map + Odom     │─────────→  Visualization   │
│       +         │ 1.5 Mbps │  + Control      │
│  Compressed Vid │─────────→  Monitoring      │
│                 │ 2-4 Mbps │                  │
└─────────────────┘         └──────────────────┘
```

**Bandwidth breakdown:**
```
FROM ROVER:
- Odometry (10 Hz):              1 KB/s    = 0.008 Mbps
- Map (0.2 Hz):                  32 KB/s   = 0.26 Mbps
- Trajectory (1 Hz):             10 KB/s   = 0.08 Mbps
- H.264 video (10fps, 720p):     400 KB/s  = 3.2 Mbps
─────────────────────────────────────────────────────
TOTAL FROM ROVER:                443 KB/s  = 3.5 Mbps ✓✓✓

TO ROVER:
- Navigation commands (10 Hz):   1 KB/s    = 0.008 Mbps
─────────────────────────────────────────────────────
TOTAL TO ROVER:                  1 KB/s    = 0.008 Mbps ✓✓✓

TOTAL BIDIRECTIONAL:             444 KB/s  = 3.5 Mbps ✓✓✓
```

This works on:
- ✅ Long-range WiFi
- ✅ 4G LTE
- ✅ Even marginal 3G

### Implementation:

```bash
# On Rover (Jetson/Pi)
ros2 launch custom_slam custom_slam.launch.py

# Also on rover, compress video for monitoring
ros2 run image_transport republish raw compressed \
  --ros-args -r in:=/camera/camera/color/image_raw \
             -r out/compressed:=/camera/compressed \
             -p compressed.jpeg_quality:=60
```

```bash
# On Base Station
# Receive map and odometry
ros2 run rviz2 rviz2

# Optionally view compressed video
ros2 run rqt_image_view rqt_image_view /camera/compressed
```

---

## Bandwidth Comparison Table

| Method | Total Bandwidth | WiFi Range | Works on 4G? | Latency | Hardware Cost |
|--------|----------------|------------|--------------|---------|---------------|
| **Raw streams** | 352 Mbps | 5-10m | ❌ | Low | $0 |
| **Compressed (30fps)** | 42 Mbps | 20-30m | ❌ | Medium | $0 |
| **Compressed (5fps)** | 8 Mbps | 50-100m | ⚠️ | Medium | $0 |
| **On-board SLAM** | 5.4 Mbps | 50-100m | ✅ | Low | $100-500 |
| **Hybrid (RECOMMENDED)** | 3.5 Mbps | 100-200m | ✅ | Low | $100-500 |

---

## Recommended Solution: Jetson Orin Nano

### Why Jetson Orin Nano?
- **Cost**: ~$199
- **Power**: 5-25W (configurable)
- **Performance**: Can easily run SLAM at 30fps
- **GPIO**: Can control motors directly
- **Compact**: Small enough for most rovers

### Modified Architecture:

```
                ROVER
    ┌─────────────────────────────┐
    │                             │
    │  ┌─────────────────┐        │
    │  │ RealSense D435i │        │
    │  └────────┬────────┘        │
    │           │                 │
    │  ┌────────▼────────┐        │
    │  │ Jetson Orin Nano│        │
    │  │                 │        │
    │  │  - Custom SLAM  │        │
    │  │  - Path Planning│        │
    │  │  - Motor Control│        │
    │  └────────┬────────┘        │
    │           │                 │
    │           ├─────────────→ Motors
    │           │                 │
    │           │ WiFi/4G         │
    └───────────┼─────────────────┘
                │ 3.5 Mbps
                ▼
    ┌───────────────────────┐
    │   Base Station        │
    │   - Visualization     │
    │   - High-level goals  │
    │   - Video monitoring  │
    └───────────────────────┘
```

### Setup on Jetson:

```bash
# 1. Install JetPack (includes ROS2)
# https://developer.nvidia.com/embedded/jetpack

# 2. Install your custom SLAM
cd ~/ros2_ws/src
# ... copy your fixed SLAM files ...
colcon build --packages-select custom_slam

# 3. Configure to start on boot
sudo systemctl enable ros2-slam.service
```

### Power Budget:
- Jetson Orin Nano (10W mode): 10W
- RealSense D435i: 3W
- WiFi module: 2W
- **Total for SLAM system: ~15W**

---

## Network Hardware Recommendations

### For 3.5 Mbps @ Long Range:

**Option 1: Long-Range WiFi** ($100-200)
- Ubiquiti Bullet M2: ~$80
- TP-Link CPE510: ~$60
- Range: 500m-2km line-of-sight
- Bandwidth: 50-150 Mbps
- ✅ Best value for outdoor rovers

**Option 2: 4G LTE Module** ($50-150)
- Waveshare SIM7600G-H: ~$50
- Quectel EC25: ~$60
- Range: Cell tower coverage (km)
- Bandwidth: 5-50 Mbps
- ✅ Best for mobile rovers

**Option 3: 5.8GHz Video Link** ($50-200)
- Used for FPV drones
- Range: 1-5km
- Bandwidth: 10-40 Mbps
- ⚠️ May need FCC license

---

## Code Modifications for On-Board Processing

### Minimal changes to your SLAM node:

```python
# In custom_slam_node.py, add parameter for remote mode

self.declare_parameter('remote_mode', False)  # False = on-board
self.remote_mode = self.get_parameter('remote_mode').value

# Reduce map publish rate when remote
if self.remote_mode:
    self.create_timer(5.0, self.publish_map)  # Only 0.2 Hz
else:
    self.create_timer(1.0, self.publish_map)  # Full 1 Hz
```

### Launch on rover:

```bash
ros2 launch custom_slam custom_slam.launch.py remote_mode:=false
```

### On base station, just subscribe to topics:

```bash
# Set ROS_DOMAIN_ID if needed
export ROS_DOMAIN_ID=0

# Launch just visualization
ros2 run rviz2 rviz2
```

---

## Quick Reference: Bandwidth by Use Case

### Indoor Lab Testing (Short Range)
- **Use**: Compressed streams (30fps)
- **Bandwidth**: 42 Mbps
- **Hardware**: Base station processes everything
- **Cost**: $0 extra

### Outdoor Competition (100m range)
- **Use**: On-board SLAM + compressed monitoring
- **Bandwidth**: 3.5 Mbps
- **Hardware**: Jetson Nano + long-range WiFi
- **Cost**: $180

### Long-Range Exploration (>500m)
- **Use**: On-board SLAM + low-rate telemetry
- **Bandwidth**: 1.5 Mbps (no video)
- **Hardware**: Jetson Nano + 4G LTE
- **Cost**: $250

### Autonomous Operation
- **Use**: Fully on-board processing
- **Bandwidth**: 0.01 Mbps (occasional status)
- **Hardware**: Jetson Orin Nano
- **Cost**: $200

---

## Summary & Recommendation

### For Your Rover, I Recommend:

**🎯 Hybrid Approach with Jetson Orin Nano:**
- Run SLAM on the rover
- Stream compressed video at 5-10fps for monitoring
- Send map updates every 5 seconds
- **Total bandwidth: 3-4 Mbps**
- Works reliably on WiFi or 4G
- Rover can operate autonomously

### Implementation Steps:
1. Get a Jetson Orin Nano ($199)
2. Install your fixed SLAM code on it
3. Set up long-range WiFi ($80 for Ubiquiti)
4. Test with incrementally longer ranges
5. Add 4G backup if needed ($50)

**Total investment: ~$330 for reliable long-range operation**

This gives you the best balance of:
- ✅ Low bandwidth usage
- ✅ Autonomous operation
- ✅ Real-time performance
- ✅ Scalable to long range
- ✅ Reasonable cost

---

Would you like me to create configuration files for the on-board setup or help you choose specific hardware for your rover?
