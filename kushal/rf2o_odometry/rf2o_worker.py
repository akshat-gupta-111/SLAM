import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import json
import socket
import math

class RF2OWorker(Node):
    def __init__(self):
        super().__init__('rf2o_worker')
        # Subscribe to the RF2O C++ node output
        self.subscription = self.create_subscription(
            Odometry,
            '/odom_rf2o',
            self.odom_callback,
            10)
        
        # Configure UDP socket to send data to ekf_server.py funnel
        self.udp_ip = "127.0.0.1"  # Localhost
        self.udp_port = 8001       # Port where EKF expects RF2O data
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        self.get_logger().info('RF2O Python Wrapper Started. Listening on /odom_rf2o...')

    def odom_callback(self, msg):
        # Extract the relevant velocities from the geometry_msgs/Twist part of the Odometry message
        # RF2O planar odometry provides linear.x, linear.y (for mecanum), and angular.z
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        omega = msg.twist.twist.angular.z
        
        # Extract nanosecond timestamp from the message header
        timestamp_ns = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
        
        # Pack into a simple dictionary payload
        payload = {
            "source": "rf2o_odometry",
            "timestamp_ns": timestamp_ns,
            "vx": vx,
            "vy": vy,
            "omega": omega
        }
        
        # Serialize to JSON and send over UDP to the EKF Hub
        try:
            json_payload = json.dumps(payload).encode('utf-8')
            self.sock.sendto(json_payload, (self.udp_ip, self.udp_port))
        except Exception as e:
            self.get_logger().error(f"Failed to send UDP payload: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = RF2OWorker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()