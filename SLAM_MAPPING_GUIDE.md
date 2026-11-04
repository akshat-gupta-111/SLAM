# SLAM Mapping — Complete Field Guide
### Mecanum Robot · ROS 2 Humble · Jetson Nano

> **Scope of this guide:** Mapping, map saving, and checkpoint creation.  
> Navigation and localisation are **halted** and not covered here.

---

## Table of Contents

1. [System Architecture](#1-system-architecture)
2. [Software Requirements](#2-software-requirements)
3. [Directory Structure](#3-directory-structure)
4. [Build & Install](#4-build--install)
5. [Full Startup Sequence](#5-full-startup-sequence-mapping-session)
6. [What Each Terminal Does](#6-what-each-terminal-does)
7. [Monitoring the Map in RViz](#7-monitoring-the-map-in-rviz)
8. [Industry Best Practices — How to Drive During Mapping](#8-industry-best-practices--how-to-drive-during-mapping)
9. [Saving the Map](#9-saving-the-map)
10. [Checkpoint Creation](#10-checkpoint-creation)
11. [Verifying Your Map](#11-verifying-your-map)
12. [Troubleshooting](#12-troubleshooting)
13. [Why the Old mapper_utility Failed](#13-why-the-old-mapper_utility-failed)
14. [TF Tree Explained](#14-tf-tree-explained)
15. [Configuration Reference](#15-configuration-reference)
16. [Halted: Navigation & Localisation](#16-halted-navigation--localisation)

---

## 1. System Architecture

```
┌──────────────────────────────────────────────────────────────────┐
│                     SENSOR LAYER                                  │
│   IMU (BNO055)            RPLiDAR A1                             │
│   /imu/data  180 Hz       /scan  10 Hz                          │
└────────┬──────────────────────────┬─────────────────────────────┘
         │                          │
         │                          ▼
         │               ┌──────────────────────────────────────┐
         │               │  rf2o_laser_odometry (C++ node)      │
         │               │  reads /scan                         │
         │               │  publishes /odom_rf2o                │
         │               │  (velocities in GLOBAL frame)        │
         │               └──────────────┬───────────────────────┘
         │                              │ /odom_rf2o
         │                              ▼
         │               ┌──────────────────────────────────────┐
         │               │  rf2o_worker.py                      │
         │               │  holonomic fix: global → robot frame │
         │               │  publishes /odom_rf2o_corrected      │
         │               │  (vx, vy, wz in ROBOT frame)         │
         │               └──────────────┬───────────────────────┘
         │                              │ /odom_rf2o_corrected
         ▼                              ▼
┌─────────────────────────────────────────────────────────────────┐
│  robot_localization EKF node                                     │
│  fuses IMU (180 Hz) + rf2o_corrected (10 Hz)                    │
│  publishes: /odometry/filtered (30 Hz)                          │
│             TF: odom → base_link                                 │
└────────┬────────────────────────────────────────────────────────┘
         │  TF: odom → base_link
         ▼
┌──────────────────────────────────────────────────────────────────┐
│  SLAM TOOLBOX (async_slam_toolbox_node)                          │
│  inputs :  /scan  +  TF odom→base_link                          │
│  outputs:  /map   +  TF map→odom                                 │
└──────────────────────────────────────────────────────────────────┘
         │  TF: map → odom (closes the tree)
         ▼
┌──────────────────────────────────────────────────────────────────┐
│  Complete TF tree during mapping                                 │
│                                                                  │
│  map ──(slam_toolbox)──▶ odom ──(EKF)──▶ base_link             │
│                                               ├──▶ imu_link     │
│                                               └──▶ laser        │
└──────────────────────────────────────────────────────────────────┘
```

**Key principle:** SLAM Toolbox does NOT use `/odometry/filtered` as a topic.  
It reads **only `/scan`** and the **TF chain** `odom → base_link`.  
The EKF publishes that TF; SLAM Toolbox consumes it implicitly.

---

## 2. Software Requirements

```bash
# Verify all packages are installed on the Jetson
ros2 pkg list | grep -E "slam_toolbox|nav2_map_server|robot_localization|rf2o"
```

If any are missing:

```bash
sudo apt update
sudo apt install ros-humble-slam-toolbox \
                 ros-humble-nav2-map-server \
                 ros-humble-robot-localization
# rf2o must be built from source (see your rf2o workspace)
```

---

## 3. Directory Structure

```
slam_node/
├── config/
│   └── slam_toolbox_mapping.yaml   ← SLAM Toolbox tuned for Jetson Nano
├── launch/
│   └── mapping.launch.py           ← starts SLAM Toolbox only
├── maps/                            ← map files saved here (.pgm + .yaml)
│   └── .gitkeep
├── checkpoints/
│   └── checkpoints.json            ← named map-frame poses
├── slam_node/
│   ├── __init__.py
│   └── checkpoint_saver.py         ← interactive TF-based CLI
├── resource/
│   └── slam_node
├── package.xml
├── setup.py
└── setup.cfg
```

---

## 4. Build & Install

```bash
# 1.  Symlink the package into your ROS 2 workspace
#     (source stays in your repo — do NOT copy/move it)
ln -s ~/Desktop/SLAM/slam_node ~/Desktop/SLAM/ros2_ws/src/slam_node

# 2.  Build (from workspace root)
cd ~/Desktop/SLAM/ros2_ws
colcon build --packages-select slam_node --symlink-install

# 3.  Source the overlay
source install/setup.bash

# (Optional) Add to .bashrc so every new terminal sees it:
echo "source ~/Desktop/SLAM/ros2_ws/install/setup.bash" >> ~/.bashrc
```

> `--symlink-install` means edits to Python files take effect **without** rebuilding.  
> The package files stay in `~/Desktop/SLAM/slam_node/` — `ros2_ws/src/` is just a symlink.

---

## 5. Full Startup Sequence (Mapping Session)

Open **four separate terminals** over SSH (use `tmux` for convenience on Jetson).

### Terminal 1 — IMU

```bash
# IMU requires sudo — must use start_imu.sh, not a ROS launch file
sudo bash ~/Desktop/SLAM/start_imu.sh
```

> ⚠️ **Important:** Always start IMU with `sudo`. Without it, `/imu/data` will be silent
> and the EKF will have no rotation data.

Verify (open a second pane):

```bash
# IMU data uses BEST_EFFORT QoS — use a custom subscriber, not ros2 topic echo
python3 -c "
import rclpy; from sensor_msgs.msg import Imu
from rclpy.qos import qos_profile_sensor_data
rclpy.init()
n = rclpy.create_node('chk')
n.create_subscription(Imu, '/imu/data',
    lambda m: print(f'wz={m.angular_velocity.z:.4f}'), qos_profile_sensor_data)
rclpy.spin(n)"
# Should print wz values at ~180 Hz
```

Or simply check the rate:

```bash
ros2 topic hz /imu/data   # should be ~180 Hz
```

---

### Terminal 2 — rf2o + EKF (odometry stack)

```bash
source ~/Desktop/SLAM/ros2_ws/install/setup.bash
ros2 launch rf2o_odometry rf2o_ekf.launch.py
# (adjust package/launch name to match your workspace)
```

This single launch starts three things in sequence:
1. `rf2o_laser_odometry` — reads `/scan`, publishes `/odom_rf2o`
2. `rf2o_worker.py` — converts global-frame velocities to robot-frame, publishes `/odom_rf2o_corrected`
3. `ekf_localization_node` — fuses IMU + rf2o_corrected, publishes `/odometry/filtered` + TF `odom→base_link`

Verify:

```bash
ros2 topic hz /odometry/filtered          # ~30 Hz
ros2 run tf2_tools view_frames            # must show odom→base_link
```

---

### Terminal 3 — SLAM Toolbox (mapping)

```bash
source ~/Desktop/SLAM/ros2_ws/install/setup.bash
ros2 launch slam_node mapping.launch.py
```

Verify:

```bash
ros2 topic hz /map                        # publishes every ~5 s
ros2 run tf2_tools view_frames            # must show map→odom→base_link
```

---

### Terminal 4 — Teleop (drive the robot)

```bash
source ~/Desktop/SLAM/ros2_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r /cmd_vel:=/cmd_vel
```

Or use your joystick / web interface.

---

### tmux one-liner (optional)

```bash
# Launch all 4 panes in one tmux session
tmux new-session -d -s slam \; \
  send-keys 'sudo bash ~/Desktop/SLAM/start_imu.sh' C-m \; \
  split-window -h \; \
  send-keys 'ros2 launch rf2o_odometry rf2o_ekf.launch.py' C-m \; \
  split-window -v \; \
  send-keys 'ros2 launch slam_node mapping.launch.py' C-m \; \
  select-pane -t 0 \; split-window -v \; \
  send-keys 'ros2 run teleop_twist_keyboard teleop_twist_keyboard' C-m \; \
  attach-session -t slam
```

---

## 6. What Each Terminal Does

| Terminal | Node | Input | Output |
|---|---|---|---|
| 1 | IMU driver (start_imu.sh) | Hardware I²C | `/imu/data` at 180 Hz (BEST_EFFORT QoS) |
| 2 | rf2o C++ node | `/scan` | `/odom_rf2o` (velocities in global frame) |
| 2 | rf2o\_worker.py | `/odom_rf2o` | `/odom_rf2o_corrected` (vx, vy in robot frame) |
| 2 | EKF node | `/imu/data` + `/odom_rf2o_corrected` | `/odometry/filtered` at 30 Hz + TF `odom→base_link` |
| 3 | SLAM Toolbox | `/scan` + TF `odom→base_link` | `/map` + TF `map→odom` |
| 4 | teleop | keyboard | `/cmd_vel` |

---

## 7. Monitoring the Map in RViz

On your **development machine** (not Jetson, to save Jetson CPU):

```bash
# On your laptop — set ROS_DOMAIN_ID and ROS_IP to match the Jetson
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

rviz2 -d $(ros2 pkg prefix slam_toolbox)/share/slam_toolbox/rviz/mapper_params_online_async.rviz
```

Add these displays if they're missing:

| Display | Topic | Notes |
|---|---|---|
| Map | `/map` | TRANSIENT_LOCAL — arrives even if RViz starts late |
| LaserScan | `/scan` | shows current sensor readings |
| TF | — | expand to see map/odom/base_link tree |
| RobotModel | — | requires URDF on param server |

> If you can't run RViz on another machine, install it on Jetson and run it locally:
> `ros2 run rviz2 rviz2` — but expect high CPU usage.

---

## 8. Industry Best Practices — How to Drive During Mapping

Following these practices produces a **clean, consistent map** with minimal drift.

### 8.1 Speed

| Scenario | Max linear speed | Max angular speed |
|---|---|---|
| Open corridor | 0.20 m/s | 0.3 rad/s |
| Near obstacles | 0.10 m/s | 0.2 rad/s |
| Tight corners | 0.05 m/s | 0.15 rad/s |

**Rule:** if the robot moves faster than the LiDAR scan rate × scan range, consecutive scans no longer overlap and the mapper loses context.

### 8.2 Coverage strategy

```
 ┌───────────────────────────────────────────────┐
 │                                               │
 │  → → → → → → → → → → → → → → → → → →        │
 │  ↑                                   ↓        │
 │  ↑  ← ← ← ← ← ← ← ← ← ← ← ← ← ← ←↓        │
 │  ↑                                   ↓        │
 │  ↑  → → → → → → → → → → → → → → → → ↓        │
 │                                               │
 └───────────────────────────────────────────────┘
```

Drive a **boustrophedon** (lawn-mower) pattern:
- Keep parallel passes ≈ 1.5 m apart (LiDAR sees ≥3 m each side).
- After the pattern, **drive back to the start point** — this creates a **loop closure**, dramatically reducing accumulated drift.

### 8.3 Loop closure

Loop closure is the most important practice for a high-quality map.

- **What it is:** When the robot revisits a previously mapped area, SLAM Toolbox recognises matching scan features and snaps the map together, eliminating odometry drift.
- **How to trigger it:** Slowly drive back through areas you've already mapped. SLAM Toolbox does this automatically.
- **Indicator in RViz:** Walls that were slightly misaligned will "click" into alignment — watch the `/map` topic.

### 8.4 Avoiding bad data

- Keep the LiDAR **unobstructed** — no cables hanging in front.
- Avoid **glass walls** and **mirrors** — LiDAR beams pass through or scatter.
- Avoid **fast rotations in place** — use gentle arcs instead.
- Make sure the robot is **fully stationary** for the first few seconds before starting to drive (lets EKF converge).

### 8.5 Environment preparation

- Move any temporary obstacles (chairs, boxes) out of their normal positions *before* mapping if you want the final map to represent the clear floor plan.
- Map with **normal lighting** — not relevant for LiDAR, but good habit if you later add cameras.

---

## 9. Saving the Map

**Industry standard tool:** `nav2_map_server map_saver_cli`

```bash
# Save map to the package maps/ directory (repo location)
ros2 run nav2_map_server map_saver_cli \
    -f ~/Desktop/SLAM/slam_node/maps/my_map \
    --ros-args -p map_subscribe_transient_local:=true
```

This creates two files:
```
~/Desktop/SLAM/slam_node/maps/my_map.pgm    ← grayscale image (254=free, 0=occupied, 205=unknown)
~/Desktop/SLAM/slam_node/maps/my_map.yaml   ← metadata (resolution, origin, occupancy thresholds)
```

### What the YAML looks like

```yaml
image: my_map.pgm
resolution: 0.05        # metres per pixel
origin: [-10.0, -10.0, 0.0]   # map origin in world frame
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
```

### Save a timestamped copy

```bash
MAP_NAME="map_$(date +%Y%m%d_%H%M%S)"
ros2 run nav2_map_server map_saver_cli \
    -f ~/Desktop/SLAM/slam_node/maps/$MAP_NAME \
    --ros-args -p map_subscribe_transient_local:=true
echo "Saved: $MAP_NAME"
```

> **Important:** Save the map **before** killing the SLAM Toolbox node.  
> Once the node stops, `/map` goes away and the map is lost.

---

## 10. Checkpoint Creation

### Why TF map→base_link, not /odometry/filtered?

| Source | Frame | Problem |
|---|---|---|
| `/odometry/filtered` | `odom` | Odom drifts over time; origin resets on reboot. A checkpoint at odom (3.2, 1.1) means nothing after the robot restarts. |
| TF `map→base_link` | `map` | Map frame is anchored to the saved `.pgm` map. A checkpoint at map (3.2, 1.1) is the same physical location every time the robot loads the map. |

**Industry rule:** always save waypoints and checkpoints in the **map frame**.

### Using the checkpoint_saver CLI

In a **fifth terminal** (while mapping is running in Terminals 1–3):

```bash
source ~/Desktop/SLAM/ros2_ws/install/setup.bash
ros2 run slam_node checkpoint_saver
```

You will see:

```
╔═══════════════════════════════════════════════════════╗
║          SLAM Checkpoint Saver  (map frame)           ║
╠═══════════════════════════════════════════════════════╣
║  s  —  Save current pose as a named checkpoint        ║
║  l  —  List all saved checkpoints                     ║
║  d  —  Delete a checkpoint                            ║
║  q  —  Quit                                           ║
╚═══════════════════════════════════════════════════════╝
checkpoint_saver>
```

**Workflow:**

1. Drive the robot to the desired location.
2. Let it sit still for 1–2 seconds (so the TF settles).
3. Type `s` and press Enter.
4. Enter a name (e.g. `home`, `dock`, `room_a_entry`).
5. Repeat for every checkpoint you need.
6. Type `l` to list and verify.
7. Type `q` to quit.

### Checkpoint JSON format

```json
{
  "home": {
    "x": 0.0,
    "y": 0.0,
    "yaw": 0.0
  },
  "dock": {
    "x": 3.2142,
    "y": -1.0873,
    "yaw": -1.5708
  },
  "room_a_entry": {
    "x": 5.6300,
    "y": 2.1100,
    "yaw": 0.0
  }
}
```

All values are in the **map frame**; yaw is in **radians**.

### Manually editing checkpoints

The file lives at:
```
~/Desktop/SLAM/slam_node/checkpoints/checkpoints.json
```

Edit it directly with any text editor. The `checkpoint_saver` node reloads it on startup.

---

## 11. Verifying Your Map

### Visual inspection (RViz)

- Free space should be **white**.
- Walls and obstacles should be **black** sharp lines.
- Unknown space (never seen by LiDAR) should be **grey**.
- There should be **no duplicate walls** or **ghosting** (sign of drift — redo with more loop closures).

### TF health check

```bash
# Should print all transforms without errors
ros2 run tf2_tools view_frames
evince frames.pdf      # view the tree diagram
```

Expected tree:
```
map
  └── odom
        └── base_link
              ├── imu_link
              └── laser
```

### Topic health check

```bash
ros2 topic list        # must include /scan, /map, /odometry/filtered, /imu/data
ros2 topic hz /scan    # ~10 Hz for RPLiDAR A1
ros2 topic hz /map     # ~0.2 Hz (every 5 s per config)
```

### Monitor live EKF output

```bash
python3 ~/Desktop/SLAM/ekf_watch.py
```

Prints a live dashboard:
```
  x=+0.000m  y=+0.000m  yaw=  +0.0°  |  vx=+0.000  vy=+0.000  wz=  +0.0°/s
```

### Map file check

```bash
# Inspect the saved map image
eog ~/Desktop/SLAM/slam_node/maps/my_map.pgm
# or
display ~/Desktop/SLAM/slam_node/maps/my_map.pgm
```

---

## 12. Troubleshooting

### Problem: Map is empty / grey dot in centre

**Root cause:** SLAM Toolbox never received valid scan data or TF.

```bash
# Check 1 — is /scan arriving?
ros2 topic echo /scan --once

# Check 2 — does the TF chain exist?
ros2 run tf2_ros tf2_echo odom base_link
# If this fails → EKF is not running (Terminal 2 not started)

# Check 3 — is SLAM Toolbox seeing scans?
ros2 topic echo /diagnostics | grep slam
```

**Fix:** Ensure Terminals 1 and 2 (IMU + EKF) are running **before** Terminal 3 (SLAM Toolbox).

---

### Problem: `/imu/data` is silent (0 Hz)

**Cause:** IMU was started without sudo, or `start_imu.sh` was not run.

```bash
# Kill everything and restart with:
sudo bash ~/Desktop/SLAM/start_imu.sh
# Then verify:
ros2 topic hz /imu/data   # must show ~180 Hz
```

---

### Problem: `transform_timeout` errors in SLAM Toolbox

```
[slam_toolbox]: Failed to compute odom pose, giving up
```

**Cause:** EKF is running but lagging, or scan QoS mismatch.

```bash
# Increase timeout in slam_toolbox_mapping.yaml:
transform_timeout: 1.0   # raise from 0.5

# Verify EKF is healthy:
ros2 topic hz /odometry/filtered   # must be ~30 Hz
```

---

### Problem: checkpoint_saver says "TF lookup failed"

```
TF lookup failed: "map" passed to lookupTransform argument target_frame does not exist.
```

**Cause:** SLAM Toolbox has not published `map→odom` yet (no scans processed).

**Fix:** Ensure Terminal 3 is running **and** the robot has received at least a few LiDAR scans. Drive a short distance.

---

### Problem: Jetson CPU at 100%, map building slowly

```bash
# Check CPU
htop

# Reduce scan frequency in slam_toolbox_mapping.yaml:
throttle_scans: 2          # process every 2nd scan
minimum_time_interval: 1.0 # wait 1 s between scans
map_update_interval: 10.0  # publish /map every 10 s
```

---

### Problem: Walls appear double / drifted

**Cause:** Not enough loop closure.

**Fix:** Drive slowly back through already-mapped areas. Watch RViz — walls "snap" into place when loop closure fires.

---

### Problem: `colcon build` fails with missing slam_toolbox

```bash
sudo apt install ros-humble-slam-toolbox
```

---

## 13. Why the Old mapper_utility Failed

For completeness, here are the two root causes diagnosed in `slam_node_1`:

### Root cause 1: launch file never started EKF

`mapping.launch.py` in the old package launched only `mapper_utility`.  
`mapper_utility.py` has this guard:

```python
def scan_callback(self, msg):
    if not self.mapping_active or not self.odom_received:
        return          # ← never processes any scan
```

`odom_received` becomes `True` only when `/odometry/filtered` arrives.  
But `/odometry/filtered` is published by the EKF, which the launch file never started.  
**Result:** every scan was silently dropped → empty grey map.

### Root cause 2: LiDAR yaw offset missing

Even if odometry had arrived, the beam angle formula was wrong:

```python
# Old (buggy) code in mapper_utility.py
beam_angle = self.robot_yaw + (msg.angle_min + i * msg.angle_increment)

# Correct formula — add the physical LiDAR mount offset
# (from TF base_link→laser: yaw = 1.5708 rad = 90°)
LIDAR_YAW_OFFSET = 1.5708   # from your URDF / TF static broadcaster
beam_angle = self.robot_yaw + LIDAR_YAW_OFFSET + (msg.angle_min + i * msg.angle_increment)
```

All obstacle hits would be placed 90° off from their true positions.

**The new approach (SLAM Toolbox) has neither bug:**
- SLAM Toolbox reads the `/scan` header's `frame_id` and looks up the TF for that frame automatically.
- It does not need a separate odometry topic — it reads TF directly.

---

## 14. TF Tree Explained

The TF (Transform) tree is the backbone of ROS 2 spatial reasoning.

```
map  ─────────────────────────── published by: slam_toolbox
  │   (where the map "sits" in the world)
  └──▶ odom  ──────────────────── published by: robot_localization EKF
         │   (where the robot started, drifts over time)
         └──▶ base_link  ─────── published by: robot_localization EKF
                │   (robot centre)
                ├──▶ imu_link  ── published by: static_transform_publisher (URDF)
                └──▶ laser  ───── published by: static_transform_publisher (URDF)
```

### Why map ≠ odom

- `odom` is the origin of wheel/LiDAR odometry — it drifts because sensors are imperfect.
- `map` is the origin of the SLAM-corrected global map — it is stable because loop closures correct it.
- The transform `map → odom` is SLAM Toolbox's best estimate of the accumulated drift correction.

### Checkpoint convention

Because `map` is stable across restarts, **checkpoints are stored in the `map` frame**.  
The checkpoint saver reads `map → base_link` (via `tf_buffer.lookup_transform('map', 'base_link', ...)`), which is the composition of the whole chain.

---

## 15. Configuration Reference

### `config/slam_toolbox_mapping.yaml` — key parameters

| Parameter | Default | Meaning |
|---|---|---|
| `mode` | `mapping` | Change to `localization` when loading a saved map |
| `resolution` | `0.05` | 5 cm/cell — good balance of detail vs. memory |
| `max_laser_range` | `12.0` | Discard beams beyond 12 m (RPLiDAR A1 max) |
| `throttle_scans` | `1` | Process every scan; raise to 2–3 if CPU is overloaded |
| `map_update_interval` | `5.0` | Publish `/map` every 5 s — saves CPU on Jetson |
| `minimum_travel_distance` | `0.10` | Only add a scan after moving 10 cm |
| `minimum_travel_heading` | `0.175` | Or after rotating 10° |
| `transform_timeout` | `0.5` | Max wait for TF (s) — raise to 1.0 if getting timeouts |
| `stack_size_to_use` | `40000000` | 40 MB for Ceres — **critical** on Jetson (low RAM) |
| `enable_interactive_mode` | `true` | Allows manual loop-closure correction in RViz |

### EKF noise settings (in `ekf_node_claude/config/ekf_localization.yaml`)

| Sensor | `noise_scale` | Meaning |
|---|---|---|
| IMU (`/imu/data`) | `1.0` | Trust IMU rotation fully |
| rf2o (`/odom_rf2o_corrected`) | `50.0` | Distrust rf2o heavily (reports fake motion when stationary) |

> `noise_scale: 50.0` means EKF treats rf2o's covariance as 50× larger than reported,
> so it blends IMU more aggressively. Do not lower this — it causes drift when stationary.

---

## 16. Halted: Navigation & Localisation

Navigation and localisation are **not implemented** in this version.

When you are ready to add them, the roadmap is:

1. **Map loading:** use `nav2_map_server` to serve the saved `.pgm` map.
2. **Localisation:** switch SLAM Toolbox to `mode: localization` (uses AMCL-equivalent particle filter to localise within the saved map).
3. **Nav2 stack:** add `nav2_bringup` with a `nav2_params.yaml` tuned for your robot's footprint and kinematics.
4. **Checkpoint navigation:** use the saved checkpoint JSON to send `NavigateToPose` action goals via the Nav2 action client.
5. **Mecanum kinematics:** configure `DiffDriveController` → `OmniDriveController` in `nav2_params.yaml` for holonomic movement.

This is a substantial step and deserves its own guide once mapping is verified.

---

*Generated for Kushal's mecanum-drive Jetson Nano robot — ROS 2 Humble.*