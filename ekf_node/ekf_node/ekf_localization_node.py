import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
import numpy as np
import math

class EKFLocalizationNode(Node):
    def __init__(self):
        super().__init__('ekf_localization_node')
        
        # 1. Declare parameters (allows reading directly from the loaded YAML configuration)
        self.declare_parameter('imu_yaw_variance', 1.0e-06)
        self.declare_parameter('imu_gz_variance', 8.7e-07)
        self.declare_parameter('odom_x_variance', 0.02)
        self.declare_parameter('odom_y_variance', 0.02)
        self.declare_parameter('process_noise_xy', 0.005)
        
        # 2. Fetch parameters loaded by the Launch file configuration
        yaw_var = self.get_parameter('imu_yaw_variance').get_parameter_value().double_value
        gz_var = self.get_parameter('imu_gz_variance').get_parameter_value().double_value
        odom_x_var = self.get_parameter('odom_x_variance').get_parameter_value().double_value
        odom_y_var = self.get_parameter('odom_y_variance').get_parameter_value().double_value
        q_xy = self.get_parameter('process_noise_xy').get_parameter_value().double_value

        # 3. State Vector initialization [x, y, yaw]
        self.x = np.zeros((3, 1))
        self.P = np.eye(3) * 0.1
        
        # 4. Process Noise Covariance (Q) - Loaded from parameter configurations
        self.Q = np.zeros((3, 3))
        self.Q[0, 0] = q_xy
        self.Q[1, 1] = q_xy
        self.Q[2, 2] = yaw_var
        
        # 5. Measurement Noise Covariances (R) using loaded calibration variables
        self.R_imu = np.zeros((2, 2))
        self.R_imu[0, 0] = yaw_var   # Heading variance
        self.R_imu[1, 1] = gz_var    # Turn speed variance
        
        self.R_odom = np.zeros((2, 2))
        self.R_odom[0, 0] = odom_x_var
        self.R_odom[1, 1] = odom_y_var
        
        # 6. Timing & Broadcasters
        self.last_time = self.get_clock().now()
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # 7. Subscriptions
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, qos_profile_sensor_data
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom_rf2o', self.odom_callback, 10
        )
        
        self.get_logger().info('EKF Node operational. Loaded calibration values directly from configuration data.')

    def predict(self, dt, yaw_rate):
        if dt <= 0.0:
            return
        yaw = self.x[2, 0]
        self.x[2, 0] = self.normalize_angle(yaw + yaw_rate * dt)
        F = np.eye(3)
        self.P = np.dot(np.dot(F, self.P), F.T) + self.Q * dt

    def imu_callback(self, msg):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        yaw_rate = msg.angular_velocity.z
        self.predict(dt, yaw_rate)
        
        q = msg.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        measured_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        z = np.array([[measured_yaw], [yaw_rate]])
        H = np.array([
            [0.0, 0.0, 1.0],
            [0.0, 0.0, 0.0]
        ])
        
        y = z - np.dot(H, self.x)
        y[0, 0] = self.normalize_angle(y[0, 0])
        
        S = np.dot(np.dot(H, self.P), H.T) + self.R_imu
        K = np.dot(np.dot(self.P, H.T), np.linalg.inv(S))
        
        self.x = self.x + np.dot(K, y)
        self.x[2, 0] = self.normalize_angle(self.x[2, 0])
        self.P = np.dot((np.eye(3) - np.dot(K, H)), self.P)
        
        self.broadcast_transform(current_time)

    def odom_callback(self, msg):
        z = np.array([[msg.pose.pose.position.x], [msg.pose.pose.position.y]])
        H = np.array([
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0]
        ])
        y = z - np.dot(H, self.x)
        S = np.dot(np.dot(H, self.P), H.T) + self.R_odom
        K = np.dot(np.dot(self.P, H.T), np.linalg.inv(S))
        
        self.x = self.x + np.dot(K, y)
        self.x[2, 0] = self.normalize_angle(self.x[2, 0])
        self.P = np.dot((np.eye(3) - np.dot(K, H)), self.P)

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def broadcast_transform(self, time_stamp):
        t = TransformStamped()
        t.header.stamp = time_stamp.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        t.transform.translation.x = float(self.x[0, 0])
        t.transform.translation.y = float(self.x[1, 0])
        t.transform.translation.z = 0.0
        
        yaw = float(self.x[2, 0])
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(yaw / 2.0)
        t.transform.rotation.w = math.cos(yaw / 2.0)
        
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = EKFLocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
