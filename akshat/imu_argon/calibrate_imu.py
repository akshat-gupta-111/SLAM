#!/usr/bin/env python3
import subprocess
import math
import yaml
import numpy as np
import sys

# MPU6050 Hardware Sensitivity Constants (+/- 2g and +/- 250 deg/s)
ACCEL_SCALE = 9.80665 / 16384.0            # LSB to m/s^2
GYRO_SCALE  = (math.pi / 180.0) / 131.0    # LSB to rad/s
GRAVITY     = 9.80665                      # m/s^2

WARMUP_SAMPLES = 100   # Discard first 0.5s while MPU6050 PLL & 21Hz DLPF settle
NUM_SAMPLES    = 4000  # 4000 samples @ ~180Hz = ~22 seconds of stationary data

def main():
    print("=========================================================")
    print(" INDUSTRY IMU CALIBRATION (WITH RPLIDAR VIBRATION ACTIVE)")
    print("=========================================================")
    print("1. Make sure RPLIDAR is spinning in another terminal!")
    print("2. Keep the robot 100% STILL on flat ground.")
    print(f"3. Discarding {WARMUP_SAMPLES} warm-up samples, then collecting {NUM_SAMPLES} samples...\n")

    process = subprocess.Popen(
        ['sudo', './imu_worker'],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        bufsize=1
    )

    accel_samples = []
    gyro_samples = []
    raw_count = 0

    try:
        for line in process.stdout:
            line = line.strip()
            if not line or line.startswith("Time_ns"):
                continue

            parts = line.split(",")
            if len(parts) != 7:
                continue

            raw_count += 1
            # Skip cold-start transient samples (0,0,0 while PLL locks)
            if raw_count <= WARMUP_SAMPLES:
                continue

            ax_raw, ay_raw, az_raw = int(parts[1]), int(parts[2]), int(parts[3])
            gx_raw, gy_raw, gz_raw = int(parts[4]), int(parts[5]), int(parts[6])

            # Convert to SI Units (m/s^2 and rad/s)
            ax = ax_raw * ACCEL_SCALE
            ay = ay_raw * ACCEL_SCALE
            az = az_raw * ACCEL_SCALE

            gx = gx_raw * GYRO_SCALE
            gy = gy_raw * GYRO_SCALE
            gz = gz_raw * GYRO_SCALE

            accel_samples.append([ax, ay, az])
            gyro_samples.append([gx, gy, gz])

            if len(accel_samples) % 500 == 0:
                print(f"Progress: {len(accel_samples)} / {NUM_SAMPLES} samples collected...")

            if len(accel_samples) >= NUM_SAMPLES:
                break

    except KeyboardInterrupt:
        print("\nCalibration aborted by user.")
        process.terminate()
        sys.exit(1)
    finally:
        process.terminate()

    accel_np = np.array(accel_samples)
    gyro_np  = np.array(gyro_samples)

    # 1. Stationary Biases (Mean offsets)
    accel_mean = np.mean(accel_np, axis=0)
    accel_bias = [
        float(accel_mean[0] - 0.0),
        float(accel_mean[1] - 0.0),
        float(accel_mean[2] - GRAVITY)
    ]

    gyro_mean = np.mean(gyro_np, axis=0)
    gyro_bias = [
        float(gyro_mean[0]),
        float(gyro_mean[1]),
        float(gyro_mean[2])
    ]

    # 2. 3x3 Covariance Matrices (flattened to 9-element row-major lists)
    accel_cov_matrix = np.cov(accel_np, rowvar=False)
    gyro_cov_matrix  = np.cov(gyro_np, rowvar=False)

    # 3. Stationary Roll/Pitch/Yaw Orientation Covariance (in rad^2)
    rolls   = np.arctan2(accel_np[:, 1] - accel_bias[1], accel_np[:, 2] - accel_bias[2])
    pitches = np.arctan2(-(accel_np[:, 0] - accel_bias[0]),
                         np.sqrt((accel_np[:, 1] - accel_bias[1])**2 + (accel_np[:, 2] - accel_bias[2])**2))

    gz_zeroed = gyro_np[:, 2] - gyro_bias[2]
    gz_std = float(np.std(gz_zeroed))
    yaw_variance = float((gz_std * 0.00555 * 180) ** 2)

    orientation_cov_matrix = np.diag([
        float(np.var(rolls)),
        float(np.var(pitches)),
        max(yaw_variance, 1e-6)
    ])

    # 4. 3-Sigma Gyro Deadband Threshold (kills stationary LiDAR vibration jitter)
    gyro_z_deadband = float(3.5 * gz_std)

    calib_data = {
        'imu_calibration': {
            'frame_id': 'imu_link',
            'z_height_from_ground_m': 0.15,
            'z_height_from_base_link_m': 0.115,
            'sample_rate_hz': 180,
            'accel_bias_m_s2': {
                'x': round(accel_bias[0], 6),
                'y': round(accel_bias[1], 6),
                'z': round(accel_bias[2], 6)
            },
            'gyro_bias_rad_s': {
                'x': round(gyro_bias[0], 6),
                'y': round(gyro_bias[1], 6),
                'z': round(gyro_bias[2], 6)
            },
            'gyro_z_deadband_rad_s': round(gyro_z_deadband, 6),
            'orientation_covariance': [round(float(x), 8) for x in orientation_cov_matrix.flatten()],
            'angular_velocity_covariance': [round(float(x), 8) for x in gyro_cov_matrix.flatten()],
            'linear_acceleration_covariance': [round(float(x), 8) for x in accel_cov_matrix.flatten()]
        }
    }

    yaml_filename = 'imu_calibration.yaml'
    with open(yaml_filename, 'w') as f:
        yaml.dump(calib_data, f, default_flow_style=False, sort_keys=False)

    print("\nCalibration Complete! Saved to:", yaml_filename)
    print("---------------------------------------------------------")
    print(f"Gyro Biases (rad/s) : gx={gyro_bias[0]:.5f}, gy={gyro_bias[1]:.5f}, gz={gyro_bias[2]:.5f}")
    print(f"Accel Biases (m/s^2): ax={accel_bias[0]:.5f}, ay={accel_bias[1]:.5f}, az={accel_bias[2]:.5f}")
    print(f"Gyro Z 3.5-Sigma Deadband: {gyro_z_deadband:.5f} rad/s")
    print("---------------------------------------------------------")

if __name__ == '__main__':
    main()
