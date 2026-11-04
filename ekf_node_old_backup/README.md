# ekf_node - planar EKF (IMU gyro + rf2o + visual odometry)

Output: `/odometry/filtered` (nav_msgs/Odometry) and TF `odom -> base_link`.

## Build & run
```bash
cd ~/ros2_ws/src && cp -r /path/to/ekf_node .
cd ~/ros2_ws && colcon build --packages-select ekf_node --symlink-install
source install/setup.bash
ros2 launch ekf_node ekf_localization.launch.py
```

## Before first run (checklist)
1. rf2o must NOT publish TF: set `publish_tf: false` in its params (EKF owns odom->base_link).
2. TF must exist for every sensor frame: `base_link -> imu_link` (you have it) and
   `base_link -> <camera frame>` (the `child_frame_id` of your visual odometry).
3. Set `visual_odom.topic` and measure `visual_odom.noise_std` in `config/ekf_localization.yaml`.
4. Use `use_sim_time:=true` when replaying a bag with `--clock`.

## Check it is healthy
```bash
ros2 topic hz /odometry/filtered          # ~30 Hz
ros2 topic echo /diagnostics --once       # per-sensor rate / age / accepted / rejected / nis_mean
ros2 service call /ekf_localization_node/reset std_srvs/srv/Trigger
```

## Tests (no ROS needed)
```bash
cd ekf_node && python3 -m pytest test/ -v
```

## Layout
| file | role |
|---|---|
| `ekf_node/geometry.py` | angle wrap, quaternion, lever-arm twist transform |
| `ekf_node/ekf_core.py` | state, motion model + Jacobian, Joseph update, gating |
| `ekf_node/fusion_manager.py` | time-ordered fusion, replay of late data, watchdogs, stats |
| `ekf_node/ekf_localization_node.py` | ROS wrapper (no filter math) |
| `config/ekf_localization.yaml` | every tunable |
