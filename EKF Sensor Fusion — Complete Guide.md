# EKF Sensor Fusion — Complete Guide

Nov 3, 2026 · @Kushal

This guide explains exactly how our robot combines IMU and LiDAR data using an EKF (Extended Kalman Filter) to know where it is — and how that feeds into future mapping and navigation.

## 🗺️ Overview — What Is EKF and Why Do We Need It?

Our robot has two sensors: an **IMU** (a chip that senses rotation and acceleration) and a **LiDAR** (a laser scanner that measures surroundings). Each sensor is useful but imperfect on its own:

- The **IMU** is very fast (180 readings/sec) but slowly **drifts** over time — like a compass that gradually points the wrong way
- The **LiDAR** gives good position estimates but is slow (10 readings/sec) and noisy — tiny vibrations produce small fake movements when stationary

The **EKF (Extended Kalman Filter)** is a smart math algorithm that takes both sensors together, figures out how much to trust each one at every moment, and produces one single clean best-guess position. Think of two imperfect witnesses — the EKF weighs both accounts and finds the most likely truth.

### The Big Picture Flow

```
IMU chip (180 Hz)
    │  publishes → /imu/data
    │
    ├───────────────────────────────────────────────────────────────────┐
                                                                    │
LiDAR (10 Hz)                                                       ▼
    │  publishes → /odom_rf2o                             ┌──────────────────┐
    │                                                     │   EKF NODE      │
    ► rf2o_worker.py ─► /odom_rf2o_corrected ────────►│  (fuses both)    │─► /odometry/filtered (30 Hz)
       [holonomic fix]                                   │  30 Hz output   │    TF: odom→base_link
                                                          └──────────────────┘
                                                                  │
                                                                  ▼
                                           SLAM Toolbox → Nav2 → Autonomous Movement
```

### The 7 Things EKF Tracks (The State Vector)

At every moment, the EKF maintains 7 numbers about the robot:

| # | Symbol | What It Means | Unit |
| --- | --- | --- | --- |
| 1 | x | Position left/right from start | meters |
| 2 | y | Position forward/backward from start | meters |
| 3 | yaw | Which direction the robot faces | radians |
| 4 | vx | Speed forward/backward in robot's own frame | m/s |
| 5 | vy | Speed sideways (strafe left/right, Mecanum only!) | m/s |
| 6 | wz | Rotation speed | rad/s |
| 7 | bgz | Gyro bias: how much the IMU is drifting, estimated live | rad/s |

The bgz (gyro bias) is what makes our EKF special — it watches how much the IMU lies and subtracts that error automatically in real time, so yaw drift is corrected without any extra hardware.

## 🧠 Sensor 1: IMU (Inertial Measurement Unit)

The IMU is a small chip on the robot that measures how the robot is **physically moving and rotating** in 3D space. Our robot uses it at **180 Hz** (180 times per second).

### What Raw Data Comes Out

The IMU publishes on ROS topic: `/imu/data` (with BEST\_EFFORT quality-of-service \[QoS\])

The message type is `sensor_msgs/Imu` and contains:

| Field | What It Measures | Used? |
| --- | --- | --- |
| `angular_velocity.z` | How fast the robot is rotating (yaw rate) | YES ✔️ |
| `angular_velocity.x/y` | Rotation on other axes (tilting) | No ✖️ |
| `linear_acceleration.x/y/z` | Raw acceleration forces | No ✖️ |
| `orientation` | Absolute orientation (if available) | No ✖️ |

### Why We Only Use angular\_velocity.z

For a ground robot that mostly stays flat:

- **Rotation around Z-axis** (left-right spin) is what matters for navigation
- **Accelerometers** (linear\_acceleration) are too noisy for reliable velocity estimation on a Mecanum robot with vibrating wheels
- The IMU is mounted at `z = 0.115m` above base\_link, connected via TF frame `imu_link`

### The IMU's Problem: Gyro Drift (Bias)

The `angular_velocity.z` reading is never perfectly zero even when the robot is standing still. It has a tiny constant error called **gyro bias (bgz)**. Over time, this error accumulates and the robot thinks it has rotated when it hasn't.

**The EKF's solution**: It adds `bgz` as a 7th variable in its state, estimates it continuously, and subtracts it from every IMU reading automatically.

### How To Check IMU Data

Because the IMU uses BEST\_EFFORT QoS, normal `ros2 topic echo` shows nothing. Use this Python script instead:

```python
# Run: python3 ~/Desktop/SLAM/ekf_watch.py
# Or subscribe manually:
import rclpy
from sensor_msgs.msg import Imu
from rclpy.qos import qos_profile_sensor_data

node = rclpy.create_node('imu_check')
sub = node.create_subscription(Imu, '/imu/data',
    lambda msg: print(f'wz = {msg.angular_velocity.z:.4f} rad/s'),
    qos_profile_sensor_data)
rclpy.spin(node)
```

> **Important**: Always run `start_imu.sh` with sudo password before starting the EKF node, or the IMU topic will be silent.

## 🔫 Sensor 2: LiDAR + rf2o Odometry

The LiDAR (Light Detection And Ranging) is a spinning laser on top of the robot. It fires laser beams in a full circle and measures how far away walls and obstacles are. But we don't use it for obstacle detection here — we use it to estimate **how far the robot has moved** by comparing two consecutive laser scans.

This technique is called **rf2o (Range Flow-based 2D Odometry)** — it detects movement by noticing how the laser pattern "flows" between frames.

### Raw Data: /odom\_rf2o (10 Hz)

The rf2o C++ node processes LiDAR scans and publishes on `/odom_rf2o`:

| Field | What It Contains |
| --- | --- |
| `pose.pose.position.x/y` | Accumulated X, Y position (global frame) |
| `pose.pose.orientation` | Accumulated heading as quaternion |
| `twist.twist.linear.x` | Forward velocity (global frame) |
| `twist.twist.angular.z` | Rotation rate |
| `twist.covariance` | All zeros (rf2o doesn't estimate uncertainty) |

**Problem with raw /odom\_rf2o**: The velocities are in the **global/world frame**, not the robot's own frame. For our Mecanum robot which can strafe sideways, we need velocities in the **robot's body frame** (local vx and vy separately).

### The Holonomic Fix: rf2o\_worker.py

This Python script sits between the raw rf2o output and the EKF. It does the coordinate transformation:

```
Global frame (world X, world Y)
        ↓  [rotation by robot's heading angle]
Local frame (robot's forward vx, robot's sideways vy)
```

The math uses the robot's current heading angle (yaw) to rotate the global velocity vector into the robot's own reference frame:

```python
vx_local = ( dx_world * cos(avg_yaw) + dy_world * sin(avg_yaw)) / dt
vy_local = (-dx_world * sin(avg_yaw) + dy_world * cos(avg_yaw)) / dt
```

**Why this matters**: Without this fix, `vy` in the EKF would always be zero — the EKF would think our Mecanum robot can never strafe!

### LiDAR Physical Offset

Our LiDAR is physically rotated **90 degrees** from the robot's forward direction. This is handled in the TF (coordinate transform) setup:

```
base_link → laser: position z=0.625m, yaw=1.5708 rad (= 90°)
```

The rf2o node reads this TF automatically, so all output is already in the correct frame.

### Corrected Output: /odom\_rf2o\_corrected

After the holonomic fix, rf2o\_worker.py publishes on `/odom_rf2o_corrected`:

| Field | What It Contains |
| --- | --- |
| `twist.twist.linear.x` | vx in robot's frame (forward/backward speed) |
| `twist.twist.linear.y` | vy in robot's frame (sideways strafe speed!) |
| `twist.twist.angular.z` | omega (rotation rate) |

### Known Issue: Noise When Stationary

Rf2o reports small non-zero velocities (e.g. `linear.x = -0.0015`) even when the robot isn't moving. This causes slow yaw drift. Mitigation in config:

- `rf2o.noise_scale: 50.0` — tells EKF to trust rf2o less
- Watchdog timeout — if rf2o data stops, EKF falls back to IMU-only

> **Note**: The LiDAR topic `/scan` is also available at 10 Hz for future use in SLAM Toolbox obstacle mapping.

## ⬇️ EKF Node — What Goes In (Inputs)

The EKF node is `ekf_localization_node.py` and it lives in `~/Desktop/SLAM/ekf_node_claude/`. It reads from two sources continuously.

### Input 1: IMU Data

| Property | Value |
| --- | --- |
| **Topic** | `/imu/data` |
| **Message Type** | `sensor_msgs/Imu` |
| **Rate** | 180 Hz |
| **QoS** | BEST\_EFFORT (sensor data) |
| **What EKF reads** | `angular_velocity.z` only (yaw rate) |
| **Config key** | `imu.topic: /imu/data` |

The IMU gives the EKF its high-frequency rotation signal. Because it updates 180 times/sec, the EKF can track fast turns smoothly. The EKF uses this to update `yaw`, `wz`, and continuously refine the gyro bias estimate `bgz`.

### Input 2: LiDAR Odometry (Holonomic Corrected)

| Property | Value |
| --- | --- |
| **Topic** | `/odom_rf2o_corrected` |
| **Message Type** | `nav_msgs/Odometry` |
| **Rate** | 10 Hz |
| **QoS** | default RELIABLE |
| **What EKF reads** | `twist.linear.x` (vx), `twist.linear.y` (vy), `twist.angular.z` (wz) |
| **Config key** | `rf2o.topic: /odom_rf2o_corrected` |

The corrected odometry gives the EKF velocity measurements in the robot's own frame. This is especially important for `vy` — the sideways strafe velocity that only Mecanum robots can have.

### What Was Disabled (And Why)

| Source | Status | Reason |
| --- | --- | --- |
| Visual odometry | Disabled | `visual_odom.enabled: false` — no camera on robot |
| IMU accelerometer | Not used | Too noisy for velocity on wheeled robot |
| Raw `/odom_rf2o` | Not used directly | Replaced by corrected version from rf2o\_worker.py |
| IMU orientation | Not used | Only yaw rate (derivative) is reliable |

### Noise Settings (How Much To Trust Each Sensor)

These are set in `config/ekf_localization.yaml`:

```yaml
imu:
  topic: /imu/data
  noise_scale: 1.0        # Trust IMU rotation fully

rf2o:
  topic: /odom_rf2o_corrected
  noise_scale: 25.0       # Distrust rf2o 25x more (it's noisy!)
  watchdog_timeout: 1.0   # If rf2o silent for 1 sec, ignore it
```

A higher `noise_scale` = EKF trusts that sensor less. We set rf2o to 50.0 because it reports fake small movements when stationary.

### How They Work Together (The Fusion Cycle)

```
[Time 0.000s]  IMU arrives   → EKF predicts new state using rotation
[Time 0.005s]  IMU arrives   → EKF predicts again
[Time 0.010s]  IMU arrives   → EKF predicts again
...
[Time 0.100s]  rf2o arrives  → EKF corrects its position estimate using velocity
...
[Time 0.200s]  rf2o arrives  → EKF corrects again
```

The IMU is like a rough sketch updated every 5ms; the LiDAR is the higher-quality correction that arrives every 100ms to keep the sketch accurate.

## ⬆️ EKF Node — What Comes Out (Outputs)

The EKF runs at **30 Hz** and produces three outputs simultaneously.

### Output 1: /odometry/filtered (Main Output)

This is the primary output your team will use for everything else.

| Field | What It Contains |
| --- | --- |
| `pose.pose.position.x` | Robot's estimated X position (meters from start) |
| `pose.pose.position.y` | Robot's estimated Y position (meters from start) |
| `pose.pose.orientation` | Robot's estimated heading as quaternion |
| `twist.twist.linear.x` | Estimated forward speed (vx) |
| `twist.twist.linear.y` | Estimated sideways speed (vy) — strafe! |
| `twist.twist.angular.z` | Estimated rotation speed (wz) |
| `pose.covariance` | How uncertain the EKF is about position (6x6 matrix) |
| `twist.covariance` | How uncertain the EKF is about velocity |

The covariance matrices are a key advantage of EKF — they tell downstream systems (like SLAM Toolbox) **how confident** to be in the pose estimate at any moment.

### Output 2: TF Transform odom → base\_link

The EKF also publishes a TF (Transform) that says: "The robot (base\_link) is at this position and orientation relative to the starting point (odom frame)."

This TF is the **standard ROS 2 way** for nodes to know where the robot is. SLAM Toolbox and Nav2 both read this TF directly.

```
TF Tree during operation:

map (future: published by SLAM Toolbox)
 └── odom  ────────────── (EKF publishes this transform)
      └── base_link  ─── (robot center)
           ├── imu_link  (IMU chip, 0.115m up)
           └── laser     (LiDAR, 0.625m up, 90° rotated)
```

**Important**: The rf2o node must have `publish_tf: False` in its config, otherwise it would fight with the EKF over who owns the `odom → base_link` transform!

### Output 3: /diagnostics

Publishes health information about the EKF: whether sensors are arriving on time, what the current state estimate is, etc. Useful for debugging.

### How To Monitor Live Output

Run the watch script:

```bash
python3 ~/Desktop/SLAM/ekf_watch.py
```

This prints a live one-line dashboard:

```
  x=+0.000m  y=+0.000m  yaw=  +0.0°  |  vx=+0.000  vy=+0.000  wz=  +0.0°/s
```

| Value | Healthy At Rest | Healthy While Moving |
| --- | --- | --- |
| x, y | Should stay near 0.00 | Changes with motion |
| yaw | May drift 0–1°/min | Changes on turns |
| vx | Near 0 (±0.01) | Positive when going forward |
| vy | Near 0 (±0.01) | Non-zero when strafing sideways |
| wz | Near 0 (±0.5°/s) | Non-zero when rotating |

## 🚀 Future: SLAM, Localization, and Navigation

The EKF output is not the end — it is the foundation that everything else is built on.

### Step 1: SLAM Toolbox — Building the Map (Do Once)

**SLAM** (Simultaneous Localization And Mapping) lets the robot learn what its environment looks like while it drives around.

- You manually drive the robot through the whole space
- SLAM Toolbox reads `/odometry/filtered` (EKF pose) + `/scan` (raw LiDAR) together
- It builds a 2D grid map of walls and obstacles in real time
- When done, you save the map as a `.pgm` image file + a `.yaml` config file

```bash
# Save map after manual drive:
ros2 run nav2_map_server map_saver_cli -f ~/Desktop/SLAM/maps/my_map
# Creates: my_map.pgm (image of the space) + my_map.yaml (resolution, origin)
```

You only need to do this once per new environment. The `.pgm` file is like a black-and-white photograph of the floor plan.

### Step 2: Nav2 Localization — Finding Where You Are (AMCL)

Once you have a map, AMCL (Adaptive Monte Carlo Localization) uses it to figure out **exactly where in that map the robot is right now**, even when turned back on from scratch.

- Loads the saved `.pgm` map
- Reads `/scan` (live LiDAR) and `/odometry/filtered` (EKF pose)
- Places hundreds of "particle" guesses of where the robot might be, then eliminates wrong ones
- Publishes the `map → odom` TF transform (the final piece of the TF tree)

### The Complete TF Tree (When Fully Running)

```
map  ───────────────── (AMCL publishes this)
  └── odom  ────────── (EKF publishes this)
       └── base_link  ── (robot center)
```

With all three transforms linked, every part of the system can now answer: "Where is the robot, in the map?" by chaining `map → odom → base_link`.

### Step 3: Nav2 Navigation — Moving to a Goal

Nav2 (Navigation 2) is the brain that actually moves the robot from Point A to Point B.

- You give it a **goal pose** (target position + orientation in the map frame)
- Nav2 plans a path around obstacles using the saved map
- It then sends velocity commands to the robot's wheel controller
- The robot follows the path, re-planning if it encounters new obstacles

### Step 4: Checkpoint Automation

For your use case (moving between set checkpoint positions automatically):

```python
# Pseudocode for checkpoint automation:
checkpoints = [
    Pose(x=1.0, y=0.0, yaw=0.0),   # Checkpoint 1
    Pose(x=2.5, y=1.5, yaw=90°),  # Checkpoint 2
    Pose(x=0.0, y=0.0, yaw=0.0),   # Home
]
for checkpoint in checkpoints:
    nav2_client.send_goal(checkpoint)   # Send goal to Nav2
    nav2_client.wait_for_result()       # Wait until robot arrives
    print(f'Reached {checkpoint}')
```

This is a ROS 2 action client call — a few dozen lines of Python.

### Complete Data Flow Summary

```
IMU (180Hz)
    │ /imu/data
    ├─────────────────────────────────┐
                                    │
                                    ▼
LiDAR (10Hz) ─► [rf2o C++] ─► [rf2o_worker.py] ─► EKF ─► /odometry/filtered
    │ /scan                                           │         (+ TF odom→base_link)
    │                                                 │
    └───────────────────────────────────────► SLAM Toolbox ─► map.pgm (save once)
                                                                 └──► AMCL ─► map→odom TF
                                                                          │
                                                                          ▼
                                                                        Nav2
                                                                          │
                                                                          ▼
                                                              Checkpoint Automation ✔️
```

### Current Status

| Component | Status |
| --- | --- |
| IMU → /imu/data | Working ✔️ |
| LiDAR → /odom\_rf2o | Working ✔️ |
| rf2o\_worker.py → /odom\_rf2o\_corrected | Ready (test warmup fix) ⏳ |
| EKF → /odometry/filtered | Working ✔️ |
| SLAM Toolbox mapping | Not yet started 🟡 |
| AMCL localization | Not yet started 🟡 |
| Nav2 navigation | Not yet started 🟡 |
| Checkpoint automation | Not yet started 🟡 |
