#!/usr/bin/env python3
import os
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


def euler_from_quaternion(x, y, z, w):
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t3, t4)


class MapperUtilityNode(Node):
    """
    On-Demand Room Mapping Node:
    - Takes /odometry/filtered (from ekf_node) and /scan (from Lidar)
    - Builds a 2D OccupancyGrid on /map in real-time
    - Can pause/resume mapping anytime via /toggle_mapping or /start_mapping & /stop_mapping
    - Saves .pgm and .yaml files via /save_map service
    """

    def __init__(self):
        super().__init__('mapper_utility')

        # Parameters
        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('resolution', 0.05)       # 5cm per cell
        self.declare_parameter('width_meters', 20.0)     # 20m x 20m grid
        self.declare_parameter('height_meters', 20.0)
        self.declare_parameter('auto_start_mapping', True)
        self.declare_parameter('map_save_dir', '~/Desktop/SLAM/slam_node/maps')
        self.declare_parameter('default_map_name', 'room_map')

        self.odom_topic = self.get_parameter('odom_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.map_topic = self.get_parameter('map_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.resolution = float(self.get_parameter('resolution').value)
        self.width_m = float(self.get_parameter('width_meters').value)
        self.height_m = float(self.get_parameter('height_meters').value)
        self.mapping_active = bool(self.get_parameter('auto_start_mapping').value)
        self.map_save_dir = os.path.expanduser(self.get_parameter('map_save_dir').value)
        self.default_map_name = self.get_parameter('default_map_name').value

        os.makedirs(self.map_save_dir, exist_ok=True)

        # Grid dimensions
        self.grid_w = int(self.width_m / self.resolution)
        self.grid_h = int(self.height_m / self.resolution)
        self.origin_x = -self.width_m / 2.0
        self.origin_y = -self.height_m / 2.0

        # Log-odds grid (0.0 = unknown)
        self.log_odds = np.zeros((self.grid_h, self.grid_w), dtype=np.float32)
        self.l_occ = 0.85
        self.l_free = -0.40
        self.l_max = 3.5
        self.l_min = -3.5

        # Current filtered odometry pose
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.odom_received = False

        # QoS for /map (Transient Local so RViz / Foxglove / Map Saver get it immediately)
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.map_pub = self.create_publisher(OccupancyGrid, self.map_topic, map_qos)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, 20
        )
        self.scan_sub = self.create_subscription(
            LaserScan, self.scan_topic, self.scan_callback, 10
        )
        self.enable_sub = self.create_subscription(
            Bool, '/mapping/enable', self.enable_callback, 10
        )

        # Services for easy control
        self.srv_start = self.create_service(Trigger, '/start_mapping', self.start_mapping_cb)
        self.srv_stop = self.create_service(Trigger, '/stop_mapping', self.stop_mapping_cb)
        self.srv_reset = self.create_service(Trigger, '/reset_map', self.reset_map_cb)
        self.srv_save = self.create_service(Trigger, '/save_map', self.save_map_cb)

        self.timer = self.create_timer(1.0, self.publish_map)
        self.tf_timer = self.create_timer(0.05, self.broadcast_map_to_odom_tf)

        self.get_logger().info(
            f"Mapper Utility Ready! Listening to {self.odom_topic} & {self.scan_topic}. "
            f"Mapping Active: {self.mapping_active}"
        )

    def broadcast_map_to_odom_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.odom_frame
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

    def odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.robot_yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
        self.odom_received = True

    def enable_callback(self, msg: Bool):
        self.mapping_active = msg.data
        state = "STARTED" if self.mapping_active else "PAUSED"
        self.get_logger().info(f"Mapping state changed to: {state}")

    def start_mapping_cb(self, request, response):
        self.mapping_active = True
        response.success = True
        response.message = "Room mapping STARTED!"
        self.get_logger().info(response.message)
        return response

    def stop_mapping_cb(self, request, response):
        self.mapping_active = False
        response.success = True
        response.message = "Room mapping PAUSED!"
        self.get_logger().info(response.message)
        return response

    def reset_map_cb(self, request, response):
        self.log_odds.fill(0.0)
        self.publish_map()
        response.success = True
        response.message = "Map cleared/reset to blank!"
        self.get_logger().info(response.message)
        return response

    def world_to_grid(self, wx, wy):
        gx = int((wx - self.origin_x) / self.resolution)
        gy = int((wy - self.origin_y) / self.resolution)
        return gx, gy

    def bresenham_line(self, x0, y0, x1, y1):
        """Returns grid cells along the ray from (x0, y0) to (x1, y1)."""
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        if dx > dy:
            err = dx / 2.0
            while x != x1:
                points.append((x, y))
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != y1:
                points.append((x, y))
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy
        return points

    def scan_callback(self, msg: LaserScan):
        if not self.mapping_active or not self.odom_received:
            return

        rx_grid, ry_grid = self.world_to_grid(self.robot_x, self.robot_y)
        if not (0 <= rx_grid < self.grid_w and 0 <= ry_grid < self.grid_h):
            return

        angle = msg.angle_min
        # Downsample rays slightly if high density for smooth CPU usage on Jetson/Mac
        step = max(1, len(msg.ranges) // 180)

        for i in range(0, len(msg.ranges), step):
            r = msg.ranges[i]
            beam_angle = self.robot_yaw + (msg.angle_min + i * msg.angle_increment)

            if math.isnan(r) or r < msg.range_min:
                continue

            hit_obstacle = True
            if math.isinf(r) or r >= msg.range_max:
                r = min(msg.range_max, 8.0)
                hit_obstacle = False

            wx = self.robot_x + r * math.cos(beam_angle)
            wy = self.robot_y + r * math.sin(beam_angle)
            gx, gy = self.world_to_grid(wx, wy)

            gx_clamped = max(0, min(self.grid_w - 1, gx))
            gy_clamped = max(0, min(self.grid_h - 1, gy))

            free_cells = self.bresenham_line(rx_grid, ry_grid, gx_clamped, gy_clamped)
            for fx, fy in free_cells:
                if 0 <= fx < self.grid_w and 0 <= fy < self.grid_h:
                    self.log_odds[fy, fx] = max(self.l_min, self.log_odds[fy, fx] + self.l_free)

            if hit_obstacle and (0 <= gx < self.grid_w and 0 <= gy < self.grid_h):
                self.log_odds[gy, gx] = min(self.l_max, self.log_odds[gy, gx] + self.l_occ)

    def build_occupancy_grid_msg(self) -> OccupancyGrid:
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.info.resolution = self.resolution
        msg.info.width = self.grid_w
        msg.info.height = self.grid_h
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.orientation.w = 1.0

        # Convert log-odds to OccupancyGrid [-1, 0..100]
        grid = np.full((self.grid_h, self.grid_w), -1, dtype=np.int8)
        grid[self.log_odds > 0.4] = 100
        grid[self.log_odds < -0.4] = 0
        msg.data = grid.flatten().tolist()
        return msg

    def publish_map(self):
        msg = self.build_occupancy_grid_msg()
        self.map_pub.publish(msg)

    def save_map_files(self, map_name=None):
        if not map_name:
            map_name = self.default_map_name

        pgm_path = os.path.join(self.map_save_dir, f"{map_name}.pgm")
        yaml_path = os.path.join(self.map_save_dir, f"{map_name}.yaml")

        # Create image array: 205 = unknown, 254 = free, 0 = occupied
        img = np.full((self.grid_h, self.grid_w), 205, dtype=np.uint8)
        img[self.log_odds < -0.4] = 254
        img[self.log_odds > 0.4] = 0

        # Flip vertically because PGM top-left is (0, height)
        img_flipped = np.flipud(img)

        with open(pgm_path, 'wb') as f:
            header = f"P5
# Created by slam_node
{self.grid_w} {self.grid_h}
255
"
            f.write(header.encode('ascii'))
            f.write(img_flipped.tobytes())

        yaml_content = (
            f"image: {map_name}.pgm
"
            f"mode: trinary
"
            f"resolution: {self.resolution}
"
            f"origin: [{self.origin_x}, {self.origin_y}, 0.0]
"
            f"negate: 0
"
            f"occupied_thresh: 0.65
"
            f"free_thresh: 0.25
"
        )
        with open(yaml_path, 'w') as f:
            f.write(yaml_content)

        return pgm_path, yaml_path

    def save_map_cb(self, request, response):
        try:
            pgm_path, yaml_path = self.save_map_files()
            response.success = True
            response.message = f"Map saved successfully: {pgm_path} & {yaml_path}"
            self.get_logger().info(response.message)
        except Exception as e:
            response.success = False
            response.message = f"Failed to save map: {str(e)}"
            self.get_logger().error(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MapperUtilityNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
