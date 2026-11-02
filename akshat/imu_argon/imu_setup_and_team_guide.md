# Robot IMU Sensor Guide (MPU6050 on Jetson Nano)

Welcome! This document explains how our robot's **IMU sensor (MPU6050)** works, what each file in this folder does, how to run it, and how to use its data later in our **Kalman Filter (EKF)**.

Everything is written in simple terms so anyone on the team can run, test, or connect to this sensor in minutes.

---

## 1. Physical Placement on the Robot

Our robot is a **66 cm tall cylinder** with a **22 cm diameter**:

* **Ground to Robot Bottom (`base_link`):** $3.5\text{ cm}$ ($0.035\text{ m}$)
* **Ground to IMU Sensor (`imu_link`):** $15.0\text{ cm}$ ($0.15\text{ m}$) — mounted right in the center of the robot.
* **Base of Robot (`base_link`) to IMU (`imu_link`):** $11.5\text{ cm}$ ($0.115\text{ m}$)

---

## 2. What Files Are in This Folder?

We split the work into four simple files so the sensor runs super fast without slowing down the Jetson Nano:

### 1. `imu_worker.cpp` ( & compiled binary `imu_worker`)

* **What it is:** A lightweight C++ program that talks directly to the IMU chip over the Jetson Nano's wires (`I2C bus 7`).
* **What it does:**
  1. Reads raw movement numbers from the sensor **180 times per second (180 Hz)**.
  2. Turns on a **built-in vibration filter (21 Hz)** inside the IMU chip itself. Why? Because when our LiDAR spins on top of the robot, the whole body vibrates slightly. This hardware filter blocks that shaking before we even read the numbers.
  3. Prints out raw data (`Time, ax, ay, az, gx, gy, gz`) immediately with zero delay.

### 2. `calibrate_imu.py` ( creates `imu_calibration.yaml`)

* **What it is:** A one-time setup script that measures the sensor's natural errors while the robot is sitting completely still (with the LiDAR spinning).
* **What it does:**
  1. **Finds the Constant Drift (Bias):** Cheap sensors aren't perfect—even when sitting completely still on a flat table, our sensor used to think it was slowly spinning at $1.44^\circ\text{ per second}$! This script measures that exact fake spin and saves it so we can subtract it out.
  2. **Finds the Shaking Threshold (Deadband):** Measures how much the sensor jiggles while the LiDAR spins, so we know when the robot is *actually* turning versus just vibrating in place.
  3. **Calculates Noise Trust Numbers (Covariance Matrices):** Measures how steady the sensor is and saves those numbers into `imu_calibration.yaml`. Later, the Kalman Filter uses these numbers to know how much to trust the IMU.

### 3. `imu_publisher_node.py` (The Main ROS 2 Node)

* **What it is:** The main Python program you run during robot operation.
* **What it does:**
  1. Starts `./imu_worker` in the background and loads `imu_calibration.yaml`.
  2. Subtracts the fake drift errors so the numbers are accurate.
  3. Locks the rotation speed to `0.0` when the robot is standing still so the angle **never drifts on its own**.
  4. Keeps all angles in **Radians** between $-\pi$ and $+\pi$ (`-3.14` to `+3.14`), which stops the angle from wildly jumping between `360` and `0`.
  5. Publishes the clean, ready-to-use data to the ROS 2 topic **`/imu/data`**.

### 4. `verify_imu.py` (Live Health Dashboard)

* **What it is:** A simple screen monitor to check if the IMU is working properly.
* **What it does:** Listens to `/imu/data` and prints a live, easy-to-read dashboard in your terminal showing whether the robot is still or rotating, its current tilt/turn angles in radians (and degrees), and gravity check.

---

## 3. How We Fixed the "Spinning While Still" Problem

Previously, the IMU angle kept counting `360 -> 0 -> 360` even when the robot wasn't moving. Here is how we solved it:

1. **Removed the Factory Offset:** Our sensor had a built-in factory error of $+0.025\text{ rad/s}$ ($+1.44^\circ/\text{s}$) on the Z-axis. Subtracting this stopped the constant phantom spinning.
2. **Killed LiDAR Vibration:** We enabled the MPU6050 chip's internal $21\text{ Hz}$ filter and added a tiny "dead-zone" (`0.00325 rad/s`). Any tiny vibration smaller than that dead-zone is treated as `0.0`, so the angle stays 100% frozen when parked.
3. **Standard ROS 2 Radian Wrapping:** Instead of wrapping between $0^\circ$ and $360^\circ$ (where tiny noise at $0^\circ$ jumps between $0.01^\circ$ and $359.99^\circ$), we centered zero in the middle: **$-\pi$ to $+\pi$ radians** ($-180^\circ$ to $+180^\circ$).

---

## 4. Step-by-Step Commands: How to Run Everything

### A. Daily Startup (Normal Use)

You do **not** need to recalibrate every time. Just run the publisher node!

**Terminal 1 — Start the IMU ROS 2 Node:**

```bash
cd ~/Desktop/SLAM/akshat/imu_argon
source /opt/ros/humble/setup.bash
sudo -v
python3 imu_publisher_node.py
```

*(Wait 1 second until you see: `Warm-up complete! Publishing live /imu/data...`)*

**Terminal 2 — Publish the IMU Position on the Robot (15 cm from ground / 11.5 cm above `base_link`):**

```bash
source /opt/ros/humble/setup.bash
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0.115 --yaw 0 --pitch 0 --roll 0 --frame-id base_link --child-frame-id imu_link
```

**Terminal 3 (Optional) — Check the Live Dashboard:**

```bash
cd ~/Desktop/SLAM/akshat/imu_argon
source /opt/ros/humble/setup.bash
python3 verify_imu.py
```

---

### B. Only If You Ever Need to Re-Compile or Re-Calibrate

*(Only do this if you physically remount the IMU sensor on the robot)*

1. Make sure the **RPLIDAR is spinning** in another terminal (`ros2 launch rplidar_ros rplidar_a1_launch.py`).
2. Keep the robot 100% still on the floor and run:

```bash
cd ~/Desktop/SLAM/akshat/imu_argon
g++ -O3 -Wall imu_worker.cpp -o imu_worker
sudo -v
python3 calibrate_imu.py
```

This will take ~20 seconds and update `imu_calibration.yaml`.

---

## 5. Guide for Teammates: What Data to Feed into the EKF

When integrating our IMU into the Extended Kalman Filter (EKF), everything you need is already being broadcast on the ROS 2 topic **`/imu/data`** (message type: `sensor_msgs/msg/Imu`).

### How to Subscribe in Python (Important QoS Note!)

Because the IMU publishes very fast (~180 times a second), it uses ROS 2's **`qos_profile_sensor_data`** (`BEST_EFFORT`). If you write a custom Python script to read `/imu/data`, subscribe like this:

```python
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

self.create_subscription(Imu, '/imu/data', self.imu_callback, qos_profile_sensor_data)
```

### Which Exact Values Should the EKF Use?

Our robot drives on a flat 2D floor. When combining the **LiDAR (`rf2o`)** and the **IMU (`/imu/data`)** inside the EKF, here is what each sensor should be responsible for:

| What You Need in EKF                              | Where to Get It From                        | Exact Field Inside `/imu/data`                                                                    | Should You Use IMU for This?                                                                                                                                               |
| :------------------------------------------------ | :------------------------------------------ | :-------------------------------------------------------------------------------------------------- | :------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Turn Speed ($\omega_z$ / `vyaw`)**    | **IMU (`/imu/data`)**               | `msg.angular_velocity.z` *(in rad/s)*                                                           | **YES (Primary!)** — The IMU updates 180 times/sec and reacts instantly when the robot turns, whereas the LiDAR only updates 10 times/sec.                          |
| **Heading Angle ($\theta$ / `yaw`)**    | **IMU (`/imu/data`)**               | `msg.orientation` *(Quaternion: x, y, z, w)*                                                    | **YES** — Already converted from radians and drift-locked when stationary.                                                                                          |
| **Forward & Sideways Speed ($v_x, v_y$)** | **LiDAR (`/odom_rf2o`)**            | *Do NOT get speed from IMU*                                                                       | **NO!** — An IMU only measures *acceleration* (`ax, ay`), not speed (`vx, vy`). Always get `vx` and `vy` from the LiDAR odometry (`/odom_rf2o`).        |
| **Noise / Trust Matrices (Covariance)**     | **Already built into `/imu/data`!** | `msg.orientation_covariancemsg.angular_velocity_covariance``msg.linear_acceleration_covariance` | **YES** — You don't need to type these numbers manually if your EKF reads standard ROS 2 messages; they are already attached to every single `/imu/data` message! |

### Quick Reference of Our Calibrated Noise Numbers (If Needed Manually)

If someone is writing a custom math script in Python and wants the exact variance (noise) numbers from `imu_calibration.yaml`:

* **Turn Speed Variance ($\omega_z$ / `gz`):** `8.7e-07` $(\text{rad/s})^2$
* **Heading Angle Variance ($\theta$ / `yaw`):** `1.0e-06` $\text{rad}^2$
* **Forward Acceleration Variance ($a_x$):** `0.001187` $(\text{m/s}^2)^2$
* **Sideways Acceleration Variance ($a_y$):** `0.000611` $(\text{m/s}^2)^2$
