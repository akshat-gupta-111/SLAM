import pandas as pd
import numpy as np

# Load your captured static data
df = pd.read_csv("imu_telemetry.csv")

print("📊 CALCULATED INDUSTRIAL NOISE PROFILE:")
print("-" * 50)

# 1. Accelerometer Variance (Measurement Noise Matrix 'R')
var_ax = df['ax'].var()
var_ay = df['ay'].var()
var_az = df['az'].var()

print(f"Accel Variance (m/s²)^2 -> X: {var_ax:.6f}, Y: {var_ay:.6f}, Z: {var_az:.6f}")

# 2. Gyroscope Variance (Process Noise Matrix 'Q')
var_gx = df['gx'].var()
var_gy = df['gy'].var()
var_gz = df['gz'].var()

print(f"Gyro Variance  (°/s)^2  -> X: {var_gx:.6f}, Y: {var_gy:.6f}, Z: {var_gz:.6f}")
