import subprocess
import sys

# MPU6050 Scale Conversions (±2g and ±250°/s defaults)
ACCEL_FACTOR = 16384.0 / 9.80665  # Converts raw data to m/s²
GYRO_FACTOR = 131.0               # Converts raw data to degrees/sec

# Launch the real-time C++ background binary
process = subprocess.Popen(['sudo', './imu_worker'], stdout=subprocess.PIPE, text=True)

try:
    for line in process.stdout:
        parts = line.strip().split(',')
        if len(parts) != 7 or parts.startswith("Time"):
            continue
        
        # Unpack binary string values into physical variables
        _, raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz = map(int, parts)
        
        # Scale values to standard SI metrics
        ax, ay, az = raw_ax / ACCEL_FACTOR, raw_ay / ACCEL_FACTOR, raw_az / ACCEL_FACTOR
        gx, gy, gz = raw_gx / GYRO_FACTOR, raw_gy / GYRO_FACTOR, raw_gz / GYRO_FACTOR

        # Clear line and print in-place to avoid terminal flooding
        output = f"\r[Accel m/s²] X: {ax:+6.2f} Y: {ay:+6.2f} Z: {az:+6.2f} | [Gyro °/s] X: {gx:+7.1f} Y: {gy:+7.1f} Z: {gz:+7.1f}"
        sys.stdout.write(output)
        sys.stdout.flush()

except KeyboardInterrupt:
    print("\nExiting IMU data pipeline listener stream.")
    process.terminate()
