#!/usr/bin/env python3
import subprocess
import math
import yaml
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion

ACCEL_SCALE = 9.80665 / 16384.0          # m/s^2 per LSB
GYRO_SCALE  = (math.pi / 180.0) / 131.0  # rad/s per LSB
WARMUP_SAMPLES = 100                     # Skip cold-start PLL transient


def wrap_to_pi(angle_rad: float) -> float:
    """Wrap any angle in radians strictly to [-pi, +pi]."""
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    """Convert Roll, Pitch, Yaw (in radians) to a ROS 2 Quaternion."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


class CalibratedImuNode(Node):
    def __init__(self):
        super().__init__('calibrated_imu_node')

        with open('imu_calibration.yaml', 'r') as f:
            calib = yaml.safe_load(f)['imu_calibration']

        self.frame_id = calib['frame_id']
        self.ax_bias = calib['accel_bias_m_s2']['x']
        self.ay_bias = calib['accel_bias_m_s2']['y']
        self.az_bias = calib['accel_bias_m_s2']['z']

        self.gx_bias = calib['gyro_bias_rad_s']['x']
        self.gy_bias = calib['gyro_bias_rad_s']['y']
        self.gz_bias = calib['gyro_bias_rad_s']['z']
        self.gz_deadband = calib['gyro_z_deadband_rad_s']

        self.orient_cov = [float(x) for x in calib['orientation_covariance']]
        self.gyro_cov   = [float(x) for x in calib['angular_velocity_covariance']]
        self.accel_cov  = [float(x) for x in calib['linear_acceleration_covariance']]

        # Use SensorDataQoS (Best Effort, Volatile) for high-rate real-time sensor streams
        self.imu_pub = self.create_publisher(Imu, '/imu/data', qos_profile_sensor_data)

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.last_ts_ns = None
        self.raw_count = 0
        self.alpha = 0.98

        self.process = subprocess.Popen(
            ['sudo', './imu_worker'],
            stdout=subprocess.PIPE,
            text=True,
            bufsize=1
        )

        self.get_logger().info("Calibrated /imu/data node active (warming up 100 samples)...")
        self.reader_thread = threading.Thread(target=self.read_loop, daemon=True)
        self.reader_thread.start()

    def read_loop(self):
        for line in self.process.stdout:
            if not rclpy.ok():
                break
            line = line.strip()
            if not line or line.startswith("Time_ns"):
                continue

            parts = line.split(",")
            if len(parts) != 7:
                continue

            self.raw_count += 1
            if self.raw_count <= WARMUP_SAMPLES:
                if self.raw_count == WARMUP_SAMPLES:
                    self.get_logger().info("Warm-up complete! Publishing live /imu/data...")
                continue

            ts_ns = int(parts[0])
            ax_raw, ay_raw, az_raw = int(parts[1]), int(parts[2]), int(parts[3])
            gx_raw, gy_raw, gz_raw = int(parts[4]), int(parts[5]), int(parts[6])

            # 1. Subtract Calibrated Biases
            ax = (ax_raw * ACCEL_SCALE) - self.ax_bias
            ay = (ay_raw * ACCEL_SCALE) - self.ay_bias
            az = (az_raw * ACCEL_SCALE) - self.az_bias

            gx = (gx_raw * GYRO_SCALE) - self.gx_bias
            gy = (gy_raw * GYRO_SCALE) - self.gy_bias
            gz = (gz_raw * GYRO_SCALE) - self.gz_bias

            # 2. Apply 3.5-Sigma Vibration Deadband on Yaw Rate
            if abs(gz) < self.gz_deadband:
                gz = 0.0

            # 3. Integrate angles in radians [-pi, pi]
            if self.last_ts_ns is not None:
                dt = (ts_ns - self.last_ts_ns) * 1e-9
                if 0.0 < dt < 0.1:
                    accel_roll  = math.atan2(ay, az)
                    accel_pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))

                    self.roll  = wrap_to_pi(self.alpha * (self.roll + gx * dt) + (1.0 - self.alpha) * accel_roll)
                    self.pitch = wrap_to_pi(self.alpha * (self.pitch + gy * dt) + (1.0 - self.alpha) * accel_pitch)
                    self.yaw   = wrap_to_pi(self.yaw + gz * dt)

            self.last_ts_ns = ts_ns

            # 4. Publish ROS 2 sensor_msgs/msg/Imu
            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id

            msg.orientation = euler_to_quaternion(self.roll, self.pitch, self.yaw)
            msg.orientation_covariance = self.orient_cov

            msg.angular_velocity.x = gx
            msg.angular_velocity.y = gy
            msg.angular_velocity.z = gz
            msg.angular_velocity_covariance = self.gyro_cov

            msg.linear_acceleration.x = ax
            msg.linear_acceleration.y = ay
            msg.linear_acceleration.z = az
            msg.linear_acceleration_covariance = self.accel_cov

            self.imu_pub.publish(msg)

    def destroy_node(self):
        if self.process:
            self.process.terminate()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CalibratedImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
