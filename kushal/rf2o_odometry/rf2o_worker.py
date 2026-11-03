import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
import math

class RF2OWorker(Node):
    def __init__(self):
        super().__init__('rf2o_worker')

        self.subscription = self.create_subscription(
            Odometry, '/odom_rf2o', self.odom_callback, 10)

        # Corrected holonomic data EKF ke liye
        self.publisher = self.create_publisher(
            Odometry, '/odom_rf2o_corrected', 10)

        self.prev_x = None
        self.prev_y = None
        self.prev_yaw = None
        self.prev_time = None

        self.get_logger().info('RF2O Holonomic Worker started → publishing /odom_rf2o_corrected')

    def get_yaw_from_quaternion(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def odom_callback(self, msg):
        curr_x   = msg.pose.pose.position.x
        curr_y   = msg.pose.pose.position.y
        curr_yaw = self.get_yaw_from_quaternion(msg.pose.pose.orientation)
        curr_time_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.prev_x is None:
            self.prev_x, self.prev_y = curr_x, curr_y
            self.prev_yaw, self.prev_time = curr_yaw, curr_time_sec
            return

        dt = curr_time_sec - self.prev_time
        if dt <= 0.0:
            return

        dx_world = curr_x - self.prev_x
        dy_world = curr_y - self.prev_y
        dyaw = (curr_yaw - self.prev_yaw + math.pi) % (2 * math.pi) - math.pi

        avg_yaw  = self.prev_yaw + dyaw / 2.0
        vx_local = ( dx_world * math.cos(avg_yaw) + dy_world * math.sin(avg_yaw)) / dt
        vy_local = (-dx_world * math.sin(avg_yaw) + dy_world * math.cos(avg_yaw)) / dt
        omega    = dyaw / dt

        # ROS Odometry message banao
        out = Odometry()
        out.header.stamp    = msg.header.stamp
        out.header.frame_id = 'odom'
        out.child_frame_id  = 'base_link'
        out.twist.twist.linear.x  = vx_local
        out.twist.twist.linear.y  = vy_local
        out.twist.twist.angular.z = omega

        self.publisher.publish(out)

        self.get_logger().info(
            f'vx={vx_local:+.3f} vy={vy_local:+.3f} wz={omega:+.3f}',
            throttle_duration_sec=1.0)

        self.prev_x, self.prev_y = curr_x, curr_y
        self.prev_yaw, self.prev_time = curr_yaw, curr_time_sec

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