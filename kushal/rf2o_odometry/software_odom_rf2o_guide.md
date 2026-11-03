# Robot LiDAR Odometry Guide (RF2O on Mecanum Drive)

Welcome! This document explains how our robot's **LiDAR Odometry System (Software Odometry)** works. We use an RPLiDAR A1 paired with the `rf2o_laser_odometry` package and a custom holonomic math wrapper to perfectly track our robot's forward, backward, sideways (strafing), and rotational movements.

Everything is written in simple terms so anyone on the team can run, test, or understand this module before we feed its data into the **Kalman Filter (EKF)**.

---

## 1. Physical Placement on the Robot (TF Tree)

Our robot is a **66 cm tall cylinder**. To calculate movement perfectly, the software needs to know exactly where the LiDAR is located relative to the ground and the chassis.

* **Ground to Chassis Bottom (`base_footprint` $\rightarrow$ `base_link`):** $3.5\text{ cm}$ ($0.035\text{ m}$)
* **Chassis to LiDAR Eye (`base_link` $\rightarrow$ `laser`):** $62.5\text{ cm}$ ($0.625\text{ m}$) up.
* **Total Height:** $0.035 + 0.625 = 0.66\text{ m}$ (Top of the robot).
* **Yaw Offset (Crucial):** Because the physical "front" of the RPLiDAR hardware was not perfectly aligned with the front of the robot chassis, we rotated the digital TF by **90 degrees left (1.5708 radians)**. This ensures that pushing the robot forward results in $+v_x$, and pushing it sideways (Mecanum strafing) results in $+v_y$.

---

## 2. What Files Are in This Folder?

The `rf2o_odometry` folder contains everything needed to launch the LiDAR, calculate the odometry, and tune the noise matrices.

### 1. `start_odom.launch.py` (The Master Launcher)

* **What it is:** The main ROS 2 launch file.
* **What it does:**
  1. Starts the physical RPLiDAR node to spin the motor and broadcast `/scan`.
  2. Broadcasts the static TF tree (Ground $\rightarrow$ Chassis $\rightarrow$ Laser).
  3. Safely delays the start of the C++ RF2O Odometry node by 3 seconds. This prevents "race conditions" where the software panics because the LiDAR motor hasn't spun up yet.

### 2. `rf2o_worker.py` (The Holonomic Math Wrapper)

* **What it is:** A lightweight Python script that acts as a live dashboard and math corrector.
* **What it does:** The default RF2O C++ node calculates movement in the global map frame. Because we have a Mecanum drive, this Python script subscribes to the raw data, applies Trigonometric rotation matrices, and isolates the true local $v_x$ (Forward) and $v_y$ (Sideways/Strafe) velocities. It then prints a live, clean terminal dashboard using a carriage return (`\r`) so you can visually test the hardware by pushing the robot.

### 3. `rf2o_config.yaml` (The EKF Trust File)

* **What it is:** The final noise covariance matrix ($R_k$) configuration file.
* **What it does:** No sensor is perfect. This file contains the exact variance numbers (e.g., $4.894 \times 10^{-6}$) calculated from testing. When we run the EKF, this YAML file tells the brain exactly how much to "trust" the LiDAR's velocity calculations when the robot slips or drifts.

### 4. `ekf_server.py` & `rf2o_raw_data.csv` (Calibration Utilities)

* **What they are:** Background utility files used for initial calibration.
* **What they do:** `ekf_server.py` listens to the network and collects exactly 5,000 samples of movement data, saving them into `rf2o_raw_data.csv`. We used this CSV to mathematically calculate the variances stored in `rf2o_config.yaml`. (You rarely need to run this again unless you buy a new LiDAR).

---

## 3. How We Fixed the "Strafe Tracking" Problem

Standard differential-drive robots cannot move sideways. Because we use Mecanum wheels, we had to heavily modify how the data is interpreted:

1. **Axis Alignment:** We fixed the $90^\circ$ physical hardware offset via the `start_odom.launch.py` TF arguments so X and Y axes don't mix.
2. **Holonomic Math:** In `rf2o_worker.py`, we take the global map difference (`dx_world`, `dy_world`) and multiply it by $\cos(\text{yaw})$ and $\sin(\text{yaw})$ to rotate the vectors back into the robot's local chassis frame, successfully calculating true Mecanum $v_y$ (strafing).

---

## 4. Step-by-Step Commands: How to Run Everything

### A. Daily Startup (Normal Use)

You just need to launch the main file.

**Terminal 1 — Start the LiDAR and Odometry Engine:**

```bash
cd ~/Desktop/SLAM/kushal/rf2o_odometry
source ~/ros2_ws/install/setup.bash
ros2 launch start_odom.launch.py
```

*(Wait 3 seconds for the LiDAR to spin up, and the RF2O node will automatically connect without errors).*

### B. Live Terminal Testing (Sanity Check)

If you want to manually push the robot and see the live velocities to ensure left/right/forward/backward signs are correct:

**Terminal 2 — Run the Live Dashboard:**

```bash
cd ~/Desktop/SLAM/kushal/rf2o_odometry
source ~/ros2_ws/install/setup.bash
python3 rf2o_worker.py
```

*Push the robot physically on the floor and watch the $v_x$, $v_y$, and $\omega$ numbers change in real-time on a single, clean line.*

---

## 5. Guide for Teammates: What Data to Feed into the EKF

When we fuse this LiDAR odometry with the IMU inside the Extended Kalman Filter (EKF), here is exactly what the LiDAR is responsible for:

| What You Need in EKF                           | Where to Get It From             | Should You Use LiDAR for This?                                                                                             |
| :--------------------------------------------- | :------------------------------- | :------------------------------------------------------------------------------------------------------------------------- |
| **Forward Speed ($v_x$)**              | **LiDAR (`/odom_rf2o`)** | **YES (Primary!)** — The LiDAR is highly accurate at tracking forward distance over time.                           |
| **Sideways Speed ($v_y$)**             | **LiDAR (`/odom_rf2o`)** | **YES (Primary!)** — Tracks our Mecanum strafing perfectly.                                                         |
| **Heading Angle ($\theta$ / `yaw`)** | **IMU (`/imu/data`)**    | **NO!** — Do not use LiDAR for absolute heading; the IMU's hardware gyroscope updating at 180Hz is vastly superior. |
| **Turn Speed ($\omega_z$)**            | **IMU (`/imu/data`)**    | **NO!** — Again, trust the IMU for rotational speed.                                                                |

### Quick Reference of Our Calibrated Noise Numbers (from `rf2o_config.yaml`)

If you are configuring the `robot_localization` node, these are the velocity variances ($R_k$ matrix) you need to inject for the LiDAR:

* **Forward Velocity Variance ($v_x$):** `0.000004894` $(\text{m/s})^2$
* **Sideways Velocity Variance ($v_y$):** `0.000003122` $(\text{m/s})^2$
* **Turn Speed Variance ($\omega_z$):** `0.000023821` $(\text{rad/s})^2$
