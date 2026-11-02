#!/usr/bin/env python3
import csv
import math
import os
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


def quaternion_to_yaw(q):
    """Convert a geometry_msgs/Quaternion into a 2D yaw angle (theta in radians)."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class OdomCsvLogger(Node):
    def __init__(self):
        super().__init__('odom_csv_logger')
        
        self.csv_filename = 'rf2o_odometry_log.csv'
        self.csv_file = open(self.csv_filename, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        # Write CSV Header
        self.csv_writer.writerow([
            'timestamp_sec',
            'x_m',
            'y_m',
            'theta_rad',
            'vx_m_s',
            'vy_m_s',
            'omega_rad_s'
        ])
        
        self.subscription = self.create_subscription(
            Odometry,
            '/odom_rf2o',
            self.odom_callback,
            10
        )
        
        self.sample_count = 0
        self.get_logger().info(
            f"Logging /odom_rf2o data to '{os.path.abspath(self.csv_filename)}'. "
            "Press Ctrl+C to stop and save."
        )

    def odom_callback(self, msg: Odometry):
        # 1. Timestamp in seconds
        t_sec = msg.header.stamp.sec + (msg.header.stamp.nanosec * 1e-9)
        
        # 2. Position (x, y) and Orientation (theta / yaw)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        theta = quaternion_to_yaw(msg.pose.pose.orientation)
        
        # 3. Linear and Angular Velocities (vx, vy, omega)
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        omega = msg.twist.twist.angular.z
        
        # Write row and flush immediately so data is never lost on Ctrl+C
        self.csv_writer.writerow([
            f"{t_sec:.6f}",
            f"{x:.6f}",
            f"{y:.6f}",
            f"{theta:.6f}",
            f"{vx:.6f}",
            f"{vy:.6f}",
            f"{omega:.6f}"
        ])
        self.csv_file.flush()
        
        self.sample_count += 1
        if self.sample_count % 50 == 0:
            self.get_logger().info(f"Logged {self.sample_count} samples...")

    def destroy_node(self):
        if not self.csv_file.closed:
            self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OdomCsvLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(f"Stopped logging. Total samples saved: {node.sample_count}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()