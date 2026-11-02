import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import numpy as np

class EKFLocalizationNode(Node):
    def __init__(self):
        super().__init__('ekf_localization_node')
        
        # Core State Tracking Vector [x, y, yaw]
        self.state = np.zeros(3)
        
        # Subscribers aligned with your hardware layout
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, qos_profile_sensor_data
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom_rf2o', self.odom_callback, 10
        )
        self.get_logger().info('EKF Node Listening to /imu/data and /odom_rf2o')

    def imu_callback(self, msg):
        # Read msg.angular_velocity.z and msg.orientation
        pass

    def odom_callback(self, msg):
        # Read translation variables
        pass
