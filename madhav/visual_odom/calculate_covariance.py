#!/usr/bin/env python3
import csv
import numpy as np
import yaml
import os

def compute_measurement_covariance(input_csv="robot_ekf_input.csv", output_yaml="ekf_covariance_output.yaml"):
    velocities = []

    try:
        with open(input_csv, mode='r') as file:
            reader = csv.reader(file)
            header = next(reader, None)  # Skip header row
            if not header:
                print("[ERROR] CSV file is empty.")
                return

            for row in reader:
                # Fix line check bug from previous snippet
                if len(row) < 6:
                    continue
                try:
                    # Extract vx (index 2), vy (index 3), wz (index 4)
                    vx = float(row[2])
                    vy = float(row[3])
                    wz = float(row[4])
                    velocities.append([vx, vy, wz])
                except ValueError:
                    continue

    except FileNotFoundError:
        print(f"[ERROR] File '{input_csv}' not found. Run record_robot_vo.py first.")
        return

    if len(velocities) < 10:
        print("[ERROR] Not enough data points recorded in CSV to compute meaningful covariance.")
        return

    data_matrix = np.array(velocities) # Shape: (N, 3) -> [vx, vy, wz]

    # Calculate 3x3 covariance matrix representing measurement noise variance (R)
    cov_matrix = np.cov(data_matrix, rowvar=False)

    print("\n" + "="*60)
    print(" COMPUTED MEASUREMENT NOISE COVARIANCE MATRIX (R)")
    print("="*60)
    print(cov_matrix)
    print("\nMatrix Layout:")
    print("  [0,0] -> Linear X Velocity Variance (sigma_vx^2)")
    print("  [1,1] -> Linear Y Velocity Variance (sigma_vy^2)")
    print("  [2,2] -> Angular Z Velocity Variance (sigma_wz^2)")
    print("="*60)

    # --- YAML EXPORT LOGIC ---
    # Expand 3x3 matrix into the 36-element flattened 6x6 array 
    # Order: [X, Y, Z, Roll, Pitch, Yaw] -> indices for [vx, vy, yaw_rate] are 0, 1, 5
    cov_6x6 = [0.0] * 36
    
    # Map elements safely (converting numpy floats to native python floats for YAML serialization)
    cov_6x6[0 * 6 + 0] = float(cov_matrix[0, 0])  # var(vx)
    cov_6x6[0 * 6 + 1] = float(cov_matrix[0, 1])  # cov(vx, vy)
    cov_6x6[0 * 6 + 5] = float(cov_matrix[0, 2])  # cov(vx, wz)

    cov_6x6[1 * 6 + 0] = float(cov_matrix[1, 0])  # cov(vy, vx)
    cov_6x6[1 * 6 + 1] = float(cov_matrix[1, 1])  # var(vy)
    cov_6x6[1 * 6 + 2] = float(cov_matrix[1, 2])  # cov(vy, wz)

    cov_6x6[5 * 6 + 0] = float(cov_matrix[2, 0])  # cov(wz, vx)
    cov_6x6[5 * 6 + 1] = float(cov_matrix[2, 1])  # cov(wz, vy)
    cov_6x6[5 * 6 + 5] = float(cov_matrix[2, 2])  # var(wz)

    yaml_data = {
        "visual_odometry_calibration": {
            "description": "Auto-generated measurement noise covariance from empirical CSV logs",
            "raw_3x3_covariance": cov_matrix.tolist(),
            "robot_localization_twist0_covariance": cov_6x6
        }
    }

    try:
        with open(output_yaml, 'w') as yaml_file:
            yaml.dump(yaml_data, yaml_file, default_flow_style=False, sort_keys=False)
        print(f"\n[SUCCESS] Covariance successfully exported to YAML: {os.path.abspath(output_yaml)}")
    except Exception as e:
        print(f"[ERROR] Failed to write YAML file: {e}")

if __name__ == "__main__":
    compute_measurement_covariance()