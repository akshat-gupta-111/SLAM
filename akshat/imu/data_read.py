import subprocess
import sys
import time

# Conversions for ±2g and ±250°/s defaults
ACCEL_FACTOR = 16384.0 / 9.80665  
GYRO_FACTOR = 131.0               

process = subprocess.Popen(['sudo', './imu_worker'], stdout=subprocess.PIPE, text=True)

# Calibration tracking arrays
cal_samples = []
cal_done = False
offsets = {"ax": 0.0, "ay": 0.0, "az": 0.0, "gx": 0.0, "gy": 0.0, "gz": 0.0}

print("⚠️ DO NOT MOVE IMU - Calibrating static offsets...")

try:
    for line in process.stdout:
        parts = line.strip().split(',')
        if len(parts) != 7 or parts[0].startswith("Time"):
            continue
        
        _, raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz = map(int, parts)
        
        # Scale to raw SI units first
        ax, ay, az = raw_ax / ACCEL_FACTOR, raw_ay / ACCEL_FACTOR, raw_az / ACCEL_FACTOR
        gx, gy, gz = raw_gx / GYRO_FACTOR, raw_gy / GYRO_FACTOR, raw_gz / GYRO_FACTOR

        # Phase 1: Collect calibration snapshots for roughly 3 seconds
        if not cal_done:
            cal_samples.append((ax, ay, az, gx, gy, gz))
            print(f"\rCollecting calibration frames: {len(cal_samples)}/500", end="")
            
            if len(cal_samples) >= 500:
                # Average out offsets (Gravity Z should normalize to target 9.80665 m/s²)
                avg = [sum(x)/len(x) for x in zip(*cal_samples)]
                offsets["ax"], offsets["ay"], offsets["az"] = avg[0], avg[1], avg[2] - 9.80665
                offsets["gx"], offsets["gy"], offsets["gz"] = avg[3], avg[4], avg[5]
                cal_done = True
                print("\n✅ Calibration complete! Initializing stream...")
            continue

        # Phase 2: Live stream with calculated baseline bias subtracted
        ax_c = ax - offsets["ax"]
        ay_c = ay - offsets["ay"]
        az_c = az - offsets["az"]
        gx_c = gx - offsets["gx"]
        gy_c = gy - offsets["gy"]
        gz_c = gz - offsets["gz"]

        # Clear line and print cleanly formatted calibrated data
        output = f"\r[Accel m/s²] X: {ax_c:+6.2f} Y: {ay_c:+6.2f} Z: {az_c:+6.2f} | [Gyro °/s] X: {gx_c:+6.1f} Y: {gy_c:+6.1f} Z: {gz_c:+6.1f}"
        sys.stdout.write(output)
        sys.stdout.flush()

except KeyboardInterrupt:
    print("\nExiting IMU data pipeline listener stream.")
    process.terminate()
