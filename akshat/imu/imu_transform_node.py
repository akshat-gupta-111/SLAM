import subprocess
import sys
import yaml
import numpy as np

# 1. Load the Configuration Profile
try:
    with open("imu_config.yaml", "r") as f:
        config = yaml.safe_load(f)
    print("✅ Configuration and extrinsics loaded successfully.")
except FileNotFoundError:
    sys.exit("❌ Error: 'imu_config.yaml' not found. Save the YAML file first.")

# Extract Scaling Factors
ACCEL_FACTOR = config["scale_factors"]["accelerometer"]["multiplier"] / config["scale_factors"]["accelerometer"]["target_gravity_mps2"]
GYRO_FACTOR = config["scale_factors"]["gyroscope"]["multiplier"]

# Extract Rigid-Body Offsets (Lever-Arm r vector in meters)
r_x = config["spatial_extrinsics"]["translation_meters"]["x"]
r_y = config["spatial_extrinsics"]["translation_meters"]["y"]
r_z = config["spatial_extrinsics"]["translation_meters"]["z"]  # 0.15m

# Extract Static Hardware Biases
offsets = config["static_calibration_offsets"]
ax_bias = offsets["accelerometer_bias"]["ax"]
ay_bias = offsets["accelerometer_bias"]["ay"]
az_bias = offsets["accelerometer_bias"]["az"]
gx_bias = offsets["gyroscope_bias"]["gx"]
gy_bias = offsets["gyroscope_bias"]["gy"]
gz_bias = offsets["gyroscope_bias"]["gz"]

# 2. Open the Low-Overhead C++ Hardware Interface
process = subprocess.Popen(['sudo', './imu_worker'], stdout=subprocess.PIPE, text=True)

last_ts = None
last_gz = 0.0

print("🚀 Kinematic Transform Active. Streaming true 'base_link' forces...\n")

try:
    for line in process.stdout:
        parts = line.strip().split(',')
        if len(parts) != 7 or parts.startswith("Time"):
            continue
        
        timestamp, raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz = map(int, parts)
        
        # Calculate time delta (dt) for angular acceleration derivation
        if last_ts is None:
            last_ts = timestamp
            continue
        dt = (timestamp - last_ts) / 1e9  
        last_ts = timestamp
        
        if dt <= 0:
            continue

        # Convert raw entries to SI units and strip out factory biases
        ax_meas = (raw_ax / ACCEL_FACTOR) - ax_bias
        ay_meas = (raw_ay / ACCEL_FACTOR) - ay_bias
        az_meas = (raw_az / ACCEL_FACTOR) - az_bias
        
        # Convert Gyro fields to Radians per Second for kinematics equations
        gx = np.radians((raw_gx / GYRO_FACTOR) - gx_bias)
        gy = np.radians((raw_gy / GYRO_FACTOR) - gy_bias)
        gz = np.radians((raw_gz / GYRO_FACTOR) - gz_bias)

        # Derive Angular Acceleration alpha (d_omega / dt) on the Z-axis
        alpha_z = (gz - last_gz) / dt
        last_gz = gz

        # -----------------------------------------------------------------
        # KINEMATIC LEVER-ARM COMPENSATION MECHANICS
        # Formula: a_true = a_measured - (alpha x r) - (w x (w x r))
        # -----------------------------------------------------------------
        # Tangential cross product components
        tangential_x = alpha_z * r_y
        tangential_y = alpha_z * r_x
        
        # Centripetal cross product coupling components (due to 15cm Z offset)
        centripetal_x = gx * gz * r_z
        centripetal_y = gy * gz * r_z
        centripetal_z = (gx**2 + gy**2) * r_z

        # Compute pure linear forces translating directly through base_link
        ax_true = ax_meas - tangential_x - centripetal_x
        ay_true = ay_meas + tangential_y - centripetal_y
        az_true = az_meas + centripetal_z  # Compensate downward vertical swing

        # Output the un-aliased tracking metrics for the SLAM backend
        output = f"\r🤖 [Base Link] Accel m/s² -> X: {ax_true:+6.2f} Y: {ay_true:+6.2f} Z: {az_true:+6.2f}"
        sys.stdout.write(output)
        sys.stdout.flush()

except KeyboardInterrupt:
    print("\nTerminating kinematics node execution pipeline.")
    process.terminate()
