#!/usr/bin/env python3
import os
import math
import heapq
import yaml
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Twist, TransformStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster


def euler_from_quaternion(x, y, z, w):
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t3, t4)


def quaternion_from_yaw(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class NavUtilityNode(Node):
    """
    All-in-One Localization & Checkpoint Navigation Node:
    1. Loads saved .pgm and .yaml map from ~/Desktop/SLAM/slam_node/maps/
    2. Localizes robot using /odometry/filtered + /scan
    3. Publishes localized telemetry to /nav (Odometry) and /nav/status (String)
    4. Plans shortest obstacle-free path (A*) on the loaded .pgm map to any checkpoint sent on /nav/goal
    5. Drives robot via /cmd_vel with real-time Lidar obstacle avoidance
    """

    def __init__(self):
        super().__init__('nav_utility')

        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('nav_topic', '/nav')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('map_save_dir', '~/Desktop/SLAM/slam_node/maps')
        self.declare_parameter('default_map_name', 'room_map')
        self.declare_parameter('max_linear_speed', 0.35)
        self.declare_parameter('max_angular_speed', 0.8)
        self.declare_parameter('goal_tolerance', 0.20)
        self.declare_parameter('obstacle_stop_distance', 0.35)

        self.odom_topic = self.get_parameter('odom_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.map_topic = self.get_parameter('map_topic').value
        self.nav_topic = self.get_parameter('nav_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.map_save_dir = os.path.expanduser(self.get_parameter('map_save_dir').value)
        self.map_name = self.get_parameter('default_map_name').value
        self.max_v = float(self.get_parameter('max_linear_speed').value)
        self.max_w = float(self.get_parameter('max_angular_speed').value)
        self.goal_tol = float(self.get_parameter('goal_tolerance').value)
        self.obs_dist = float(self.get_parameter('obstacle_stop_distance').value)

        self.map_loaded = False
        self.grid = None
        self.inflated_grid = None
        self.grid_w = 0
        self.grid_h = 0
        self.resolution = 0.05
        self.origin_x = -10.0
        self.origin_y = -10.0

        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.map_offset_x = 0.0
        self.map_offset_y = 0.0
        self.map_offset_yaw = 0.0

        self.waypoints = []
        self.current_wp_idx = 0
        self.goal_yaw = None
        self.navigating = False
        self.front_obstacle = False
        self.avoid_turn_dir = 0.0

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.map_pub = self.create_publisher(OccupancyGrid, self.map_topic, map_qos)
        self.nav_odom_pub = self.create_publisher(Odometry, self.nav_topic, 20)
        self.nav_status_pub = self.create_publisher(String, '/nav/status', 10)
        self.path_pub = self.create_publisher(Path, '/nav/path', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 20)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_cb, 20)
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_cb, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/nav/goal', self.goal_cb, 10)
        self.rviz_goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)

        self.stop_srv = self.create_service(Trigger, '/nav/cancel', self.cancel_nav_cb)

        self.load_saved_map()

        self.control_timer = self.create_timer(0.1, self.control_loop)
        self.map_timer = self.create_timer(2.0, self.publish_loaded_map)

        self.get_logger().info(
            f"Navigation & Localization Ready! Listening to {self.odom_topic}, publishing to {self.nav_topic} & {self.cmd_vel_topic}"
        )

    def load_saved_map(self):
        yaml_path = os.path.join(self.map_save_dir, f"{self.map_name}.yaml")
        if not os.path.exists(yaml_path):
            self.get_logger().warn(
                f"Saved map not found at {yaml_path}. Using open-space fallback until a map is saved."
            )
            return

        try:
            with open(yaml_path, 'r') as f:
                meta = yaml.safe_load(f)
            pgm_file = meta.get('image', f"{self.map_name}.pgm")
            pgm_path = os.path.join(self.map_save_dir, pgm_file)
            self.resolution = float(meta.get('resolution', 0.05))
            origin = meta.get('origin', [-10.0, -10.0, 0.0])
            self.origin_x = float(origin[0])
            self.origin_y = float(origin[1])

            with open(pgm_path, 'rb') as f:
                magic = f.readline().decode('ascii').strip()
                line = f.readline().decode('ascii').strip()
                while line.startswith('#'):
                    line = f.readline().decode('ascii').strip()
                w, h = map(int, line.split())
                maxval = int(f.readline().decode('ascii').strip())
                raw = np.frombuffer(f.read(), dtype=np.uint8).reshape((h, w))

            raw_unflipped = np.flipud(raw)
            self.grid_w = w
            self.grid_h = h
            self.grid = np.full((h, w), -1, dtype=np.int8)
            self.grid[raw_unflipped >= 230] = 0
            self.grid[raw_unflipped <= 50] = 100

            self.inflated_grid = self.grid.copy()
            occ_y, occ_x = np.where(self.grid == 100)
            for dy in range(-3, 4):
                for dx in range(-3, 4):
                    ny = np.clip(occ_y + dy, 0, h - 1)
                    nx = np.clip(occ_x + dx, 0, w - 1)
                    self.inflated_grid[ny, nx] = 100

            self.map_loaded = True
            self.publish_loaded_map()
            self.get_logger().info(f"Loaded map '{pgm_path}' ({w}x{h} @ {self.resolution}m/px)")
        except Exception as e:
            self.get_logger().error(f"Error loading map: {e}")

    def publish_loaded_map(self):
        if not self.map_loaded:
            return
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info.resolution = self.resolution
        msg.info.width = self.grid_w
        msg.info.height = self.grid_h
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.orientation.w = 1.0
        msg.data = self.grid.flatten().tolist()
        self.map_pub.publish(msg)

    def get_localized_pose(self):
        lx = self.odom_x + self.map_offset_x
        ly = self.odom_y + self.map_offset_y
        lyaw = normalize_angle(self.odom_yaw + self.map_offset_yaw)
        return lx, ly, lyaw

    def odom_cb(self, msg: Odometry):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.odom_yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)

        lx, ly, lyaw = self.get_localized_pose()

        nav_msg = Odometry()
        nav_msg.header.stamp = self.get_clock().now().to_msg()
        nav_msg.header.frame_id = 'map'
        nav_msg.child_frame_id = 'base_link'
        nav_msg.pose.pose.position.x = lx
        nav_msg.pose.pose.position.y = ly
        qx, qy, qz, qw = quaternion_from_yaw(lyaw)
        nav_msg.pose.pose.orientation.x = qx
        nav_msg.pose.pose.orientation.y = qy
        nav_msg.pose.pose.orientation.z = qz
        nav_msg.pose.pose.orientation.w = qw
        nav_msg.twist = msg.twist
        self.nav_odom_pub.publish(nav_msg)

        t = TransformStamped()
        t.header.stamp = nav_msg.header.stamp
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.translation.x = self.map_offset_x
        t.transform.translation.y = self.map_offset_y
        oqx, oqy, oqz, oqw = quaternion_from_yaw(self.map_offset_yaw)
        t.transform.rotation.x = oqx
        t.transform.rotation.y = oqy
        t.transform.rotation.z = oqz
        t.transform.rotation.w = oqw
        self.tf_broadcaster.sendTransform(t)

    def scan_cb(self, msg: LaserScan):
        front_ranges = []
        left_ranges = []
        right_ranges = []

        for i, r in enumerate(msg.ranges):
            if math.isnan(r) or math.isinf(r) or r < msg.range_min:
                continue
            ang = normalize_angle(msg.angle_min + i * msg.angle_increment)
            if -0.52 <= ang <= 0.52:
                front_ranges.append(r)
            elif 0.52 < ang <= 1.57:
                left_ranges.append(r)
            elif -1.57 <= ang < -0.52:
                right_ranges.append(r)

        min_front = min(front_ranges) if front_ranges else 10.0
        if min_front < self.obs_dist:
            self.front_obstacle = True
            avg_left = np.mean(left_ranges) if left_ranges else 0.0
            avg_right = np.mean(right_ranges) if right_ranges else 0.0
            self.avoid_turn_dir = 1.0 if avg_left >= avg_right else -1.0
        else:
            self.front_obstacle = False

    def world_to_grid(self, wx, wy):
        gx = int((wx - self.origin_x) / self.resolution)
        gy = int((wy - self.origin_y) / self.resolution)
        return gx, gy

    def grid_to_world(self, gx, gy):
        wx = self.origin_x + (gx + 0.5) * self.resolution
        wy = self.origin_y + (gy + 0.5) * self.resolution
        return wx, wy

    def plan_astar_path(self, start_xy, goal_xy):
        if not self.map_loaded:
            return [goal_xy]

        sx, sy = self.world_to_grid(start_xy[0], start_xy[1])
        gx, gy = self.world_to_grid(goal_xy[0], goal_xy[1])

        if not (0 <= sx < self.grid_w and 0 <= sy < self.grid_h):
            return [goal_xy]
        if not (0 <= gx < self.grid_w and 0 <= gy < self.grid_h):
            return [goal_xy]

        open_set = []
        heapq.heappush(open_set, (0.0, (sx, sy)))
        came_from = {}
        g_score = {(sx, sy): 0.0}

        neighbors = [
            (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
            (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)
        ]

        visited = 0
        max_iter = 25000

        while open_set and visited < max_iter:
            visited += 1
            _, current = heapq.heappop(open_set)
            if current == (gx, gy):
                path_cells = []
                while current in came_from:
                    path_cells.append(current)
                    current = came_from[current]
                path_cells.reverse()
                waypoints = [self.grid_to_world(cx, cy) for cx, cy in path_cells[::4]]
                waypoints.append(goal_xy)
                return waypoints

            cx, cy = current
            for dx, dy, cost in neighbors:
                nx, ny = cx + dx, cy + dy
                if not (0 <= nx < self.grid_w and 0 <= ny < self.grid_h):
                    continue
                if self.inflated_grid[ny, nx] == 100 and (nx, ny) != (gx, gy):
                    continue
                tentative_g = g_score[current] + cost
                if tentative_g < g_score.get((nx, ny), float('inf')):
                    came_from[(nx, ny)] = current
                    g_score[(nx, ny)] = tentative_g
                    h_cost = math.hypot(gx - nx, gy - ny)
                    heapq.heappush(open_set, (tentative_g + h_cost, (nx, ny)))

        self.get_logger().warn("A* fallback: direct line to checkpoint")
        return [goal_xy]

    def goal_cb(self, msg: PoseStamped):
        gx = msg.pose.position.x
        gy = msg.pose.position.y
        q = msg.pose.orientation
        self.goal_yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)

        lx, ly, _ = self.get_localized_pose()
        self.waypoints = self.plan_astar_path((lx, ly), (gx, gy))
        self.current_wp_idx = 0
        self.navigating = True

        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        for wx, wy in self.waypoints:
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = wx
            ps.pose.position.y = wy
            ps.pose.orientation.w = 1.0
            path_msg.poses.append(ps)
        self.path_pub.publish(path_msg)

        status = String()
        status.data = f"NAVIGATING to ({gx:.2f}, {gy:.2f}) via {len(self.waypoints)} waypoints"
        self.nav_status_pub.publish(status)
        self.get_logger().info(status.data)

    def cancel_nav_cb(self, request, response):
        self.navigating = False
        self.waypoints = []
        self.cmd_vel_pub.publish(Twist())
        response.success = True
        response.message = "Navigation cancelled and robot stopped."
        self.get_logger().info(response.message)
        return response

    def control_loop(self):
        if not self.navigating or not self.waypoints:
            return

        lx, ly, lyaw = self.get_localized_pose()
        tx, ty = self.waypoints[self.current_wp_idx]
        dist = math.hypot(tx - lx, ty - ly)

        if dist < self.goal_tol:
            if self.current_wp_idx < len(self.waypoints) - 1:
                self.current_wp_idx += 1
                return
            else:
                self.navigating = False
                self.cmd_vel_pub.publish(Twist())
                status = String()
                status.data = f"REACHED CHECKPOINT ({tx:.2f}, {ty:.2f})!"
                self.nav_status_pub.publish(status)
                self.get_logger().info(status.data)
                return

        cmd = Twist()
        if self.front_obstacle:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5 * self.avoid_turn_dir
            self.cmd_vel_pub.publish(cmd)
            return

        target_yaw = math.atan2(ty - ly, tx - lx)
        yaw_err = normalize_angle(target_yaw - lyaw)

        if abs(yaw_err) > 0.45:
            cmd.linear.x = 0.05
            cmd.angular.z = max(-self.max_w, min(self.max_w, 1.5 * yaw_err))
        else:
            cmd.linear.x = min(self.max_v, 0.8 * dist)
            cmd.angular.z = max(-self.max_w, min(self.max_w, 1.2 * yaw_err))

        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = NavUtilityNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_vel_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
