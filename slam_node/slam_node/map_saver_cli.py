#!/usr/bin/env python3
import os
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid


class StandaloneMapSaver(Node):
    """
    One-click CLI utility to grab the latest /map topic
    and save it directly as .pgm and .yaml files!
    Usage: ros2 run slam_node save_map [map_name]
    """

    def __init__(self, map_name='room_map', save_dir='~/Desktop/SLAM/slam_node/maps'):
        super().__init__('standalone_map_saver')
        self.map_name = map_name
        self.save_dir = os.path.expanduser(save_dir)
        os.makedirs(self.save_dir, exist_ok=True)
        self.saved = False

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.sub = self.create_subscription(OccupancyGrid, '/map', self.map_cb, map_qos)
        self.get_logger().info(f"Waiting for /map topic to save as '{self.map_name}.pgm'...")

    def map_cb(self, msg: OccupancyGrid):
        if self.saved:
            return
        w = msg.info.width
        h = msg.info.height
        res = msg.info.resolution
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y

        data = np.array(msg.data, dtype=np.int8).reshape((h, w))
        img = np.full((h, w), 205, dtype=np.uint8)
        img[data == 0] = 254
        img[data > 50] = 0
        img_flipped = np.flipud(img)

        pgm_path = os.path.join(self.save_dir, f"{self.map_name}.pgm")
        yaml_path = os.path.join(self.save_dir, f"{self.map_name}.yaml")

        with open(pgm_path, 'wb') as f:
            header = f"P5\n# Saved by slam_node save_map\n{w} {h}\n255\n"
            f.write(header.encode('ascii'))
            f.write(img_flipped.tobytes())

        yaml_lines = [
            f"image: {self.map_name}.pgm",
            "mode: trinary",
            f"resolution: {res}",
            f"origin: [{ox}, {oy}, 0.0]",
            "negate: 0",
            "occupied_thresh: 0.65",
            "free_thresh: 0.25",
        ]
        with open(yaml_path, 'w') as f:
            f.write("\n".join(yaml_lines) + "\n")

        self.get_logger().info(f"SUCCESS! Saved PGM: {pgm_path}")
        self.get_logger().info(f"SUCCESS! Saved YAML: {yaml_path}")
        self.saved = True


def main(args=None):
    rclpy.init(args=args)
    map_name = sys.argv[1] if len(sys.argv) > 1 else 'room_map'
    node = StandaloneMapSaver(map_name=map_name)
    while rclpy.ok() and not node.saved:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
