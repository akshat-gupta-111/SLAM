#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


def quaternion_to_euler(q):
    """Convert ROS Quaternion to Roll, Pitch, Yaw in Radians [-pi, +pi]."""
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class ImuMonitor(Node):
    def __init__(self):
        super().__init__('imu_monitor')
        # Match SensorDataQoS (Best Effort) from the publisher
        self.sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_cb,
            qos_profile_sensor_data
        )
        self.msg_count = 0

    def imu_cb(self, msg: Imu):
        self.msg_count += 1
        # Print update every 36 messages (~5 times per second at 180Hz)
        if self.msg_count % 36 != 0:
            return

        roll, pitch, yaw = quaternion_to_euler(msg.orientation)

        gx = msg.angular_velocity.x
        gy = msg.angular_velocity.y
        gz = msg.angular_velocity.z

        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z

        is_stationary = (abs(gz) == 0.0)
        gravity_ok = (9.5 <= az <= 10.1)

        status = "STILL (LOCKED - NO DRIFT)" if is_stationary else "ROTATING"
        grav_status = "OK (~9.81 m/s^2)" if gravity_ok else "CHECK TILT/SCALE"

        print("\033[2J\033[H", end="")  # Clear terminal screen for clean live view
        print("==========================================================")
        print(f" LIVE /imu/data MONITOR (Samples received: {self.msg_count})")
        print("==========================================================")
        print(f" STATUS           : {status}")
        print(f" GRAVITY CHECK    : {grav_status} (az = {az:+.3f} m/s^2)")
        print("----------------------------------------------------------")
        print(" ANGLES (Radians [-pi to +pi])   |   (Degrees for reference)")
        print(f"   Roll  (X-tilt) : {roll:+.4f} rad   |   ({math.degrees(roll):+6.2f} deg)")
        print(f"   Pitch (Y-tilt) : {pitch:+.4f} rad   |   ({math.degrees(pitch):+6.2f} deg)")
        print(f"   Yaw   (Z-turn) : {yaw:+.4f} rad   |   ({math.degrees(yaw):+6.2f} deg)")
        print("----------------------------------------------------------")
        print(" ANGULAR VELOCITY (rad/s)")
        print(f"   gx: {gx:+.4f}  |  gy: {gy:+.4f}  |  gz (Yaw Rate): {gz:+.4f}")
        print("----------------------------------------------------------")
        print(" LINEAR ACCELERATION (m/s^2)")
        print(f"   ax: {ax:+.4f}  |  ay: {ay:+.4f}  |  az (Gravity) : {az:+.4f}")
        print("==========================================================")
        print("Press Ctrl+C to exit.")


def main(args=None):
    rclpy.init(args=args)
    node = ImuMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
