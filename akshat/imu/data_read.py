import subprocess
import sys
import json
import csv

# Scale multipliers
ACCEL_FACTOR = 16384.0 / 9.80665  
GYRO_FACTOR = 131.0               

process = subprocess.Popen(['sudo', './imu_worker'], stdout=subprocess.PIPE, text=True)

cal_samples, data_log = [], []
cal_done = False
offsets = {"ax": 0.0, "ay": 0.0, "az": 0.0, "gx": 0.0, "gy": 0.0, "gz": 0.0}

print("⚠️ DO NOT MOVE IMU - Calibrating static offsets...")

try:
    for line in process.stdout:
        parts = line.strip().split(',')
        if len(parts) != 7 or parts[0].startswith("Time"):
            continue
        
        timestamp, raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz = map(int, parts)
        
        ax, ay, az = raw_ax / ACCEL_FACTOR, raw_ay / ACCEL_FACTOR, raw_az / ACCEL_FACTOR
        gx, gy, gz = raw_gx / GYRO_FACTOR, raw_gy / GYRO_FACTOR, raw_gz / GYRO_FACTOR

        if not cal_done:
            cal_samples.append((ax, ay, az, gx, gy, gz))
            print(f"\rCollecting calibration frames: {len(cal_samples)}/500", end="")
            if len(cal_samples) >= 500:
                avg = [sum(x)/len(x) for x in zip(*cal_samples)]
                offsets["ax"], offsets["ay"], offsets["az"] = avg[0], avg[1], avg[2] - 9.80665
                offsets["gx"], offsets["gy"], offsets["gz"] = avg[3], avg[4], avg[5]
                cal_done = True
                print("\n✅ Calibration complete! Saving profile and logging stream. Press Ctrl+C to stop...\n")
                
                # Instantly dump the calibration file
                with open("imu_calibration.json", "w") as f:
                    json.dump(offsets, f, indent=4)
            continue

        # Subtract baseline offsets
        ax_c, ay_c, az_c = ax - offsets["ax"], ay - offsets["ay"], az - offsets["az"]
        gx_c, gy_c, gz_c = gx - offsets["gx"], gy - offsets["gy"], gz - offsets["gz"]

        # Track history for SLAM/Graphing analysis
        data_log.append([timestamp, ax_c, ay_c, az_c, gx_c, gy_c, gz_c])

        output = f"[Accel m/s²] X: {ax_c:+6.2f} Y: {ay_c:+6.2f} Z: {az_c:+6.2f} | [Gyro °/s] X: {gx_c:+6.1f} Y: {gy_c:+6.1f} Z: {gz_c:+6.1f}"
        print(output)

except KeyboardInterrupt:
    print("\n\nStopping stream. Saving captured datasets...")
    process.terminate()
    
    # Save the runtime logs to a CSV sheet
    if data_log:
        with open("imu_telemetry.csv", "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["Timestamp_ns", "ax", "ay", "az", "gx", "gy", "gz"])
            writer.writerows(data_log)
        print(f"💾 Saved {len(data_log)} clean samples to 'imu_telemetry.csv'")
