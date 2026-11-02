import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
import math

class EKFWatch(Node):
    def __init__(self):
        super().__init__('ekf_watch')
        self.create_subscription(Odometry, '/odometry/filtered',
                                 self.cb, qos_profile_sensor_data)

    def cb(self, msg):
        x   = msg.pose.pose.position.x
        y   = msg.pose.pose.position.y
        q   = msg.pose.pose.orientation
        yaw = math.degrees(2 * math.atan2(q.z, q.w))
        vx  = msg.twist.twist.linear.x
        vy  = msg.twist.twist.linear.y
        wz  = math.degrees(msg.twist.twist.angular.z)
        print(f"\r  x={x:+.3f}m  y={y:+.3f}m  yaw={yaw:+6.1f}°  |  vx={vx:+.3f}  vy={vy:+.3f}  wz={wz:+5.1f}°/s   ", end='')

rclpy.init()
rclpy.spin(EKFWatch())