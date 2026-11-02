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
        self.udp_ip = "127.0.0.1"
        self.udp_port = 8001
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # Keep track of previous state to calculate True Velocity
        self.prev_x = None
        self.prev_y = None
        self.prev_yaw = None
        self.prev_time = None
        
        self.get_logger().info('RF2O Python Wrapper (HOLONOMIC MODE) Started. Calculating true vy...')

    def get_yaw_from_quaternion(self, q):
        # Math to extract 2D rotation (Yaw) from 3D Quaternion
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def odom_callback(self, msg):
        # 1. Read the PERFECT Pose data (x, y, yaw) from the C++ node
        curr_x = msg.pose.pose.position.x
        curr_y = msg.pose.pose.position.y
        curr_yaw = self.get_yaw_from_quaternion(msg.pose.pose.orientation)
        
        curr_time_ns = msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec
        curr_time_sec = curr_time_ns / 1e9
        
        # 2. First frame initialization
        if self.prev_x is None:
            self.prev_x = curr_x
            self.prev_y = curr_y
            self.prev_yaw = curr_yaw
            self.prev_time = curr_time_sec
            return
            
        dt = curr_time_sec - self.prev_time
        if dt <= 0.0:
            return
            
        # 3. Calculate distance moved in the GLOBAL map frame
        dx_world = curr_x - self.prev_x
        dy_world = curr_y - self.prev_y
        dyaw = curr_yaw - self.prev_yaw
        
        # Normalize yaw change to be safely within -pi to pi
        dyaw = (dyaw + math.pi) % (2 * math.pi) - math.pi
        
        # 4. Rotate the World Velocities back into the Robot's LOCAL frame
        # This accurately calculates the true Mecanum strafe (vy)
        avg_yaw = self.prev_yaw + (dyaw / 2.0)
        
        vx_local = (dx_world * math.cos(avg_yaw) + dy_world * math.sin(avg_yaw)) / dt
        vy_local = (-dx_world * math.sin(avg_yaw) + dy_world * math.cos(avg_yaw)) / dt
        omega = dyaw / dt
        
        # 5. Pack and send to the EKF Server
        payload = {
            "source": "rf2o_odometry",
            "timestamp_ns": int(curr_time_ns),
            "vx": vx_local,
            "vy": vy_local,      # Boom! Now we have real strafing data!
            "omega": omega
        }
        
        try:
            self.sock.sendto(json.dumps(payload).encode('utf-8'), (self.udp_ip, self.udp_port))
        except Exception as e:
            self.get_logger().error(f"Failed to send UDP payload: {e}")
            
        # 6. Save current state for the next calculation
        self.prev_x = curr_x
        self.prev_y = curr_y
        self.prev_yaw = curr_yaw
        self.prev_time = curr_time_sec

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