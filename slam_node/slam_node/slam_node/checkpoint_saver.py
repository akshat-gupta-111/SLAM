#!/usr/bin/env python3
"""
checkpoint_saver.py — Industry-standard checkpoint recorder for ROS 2 mapping

Industry practice:
  • Checkpoints are recorded in the MAP frame (not odom frame).
  • Position is read from the TF transform  map → base_link  (not /odometry/filtered).
  • This makes checkpoints Nav2-compatible: they survive restarts because they are
    anchored to the persistent saved map, not to a drifting odom origin.

Usage (while mapping is running):
    ros2 run slam_node checkpoint_saver

Interactive CLI:
    [s] save  — name + record current map-frame pose
    [l] list  — show all saved checkpoints
    [d] delete — remove a checkpoint by name
    [q] quit  — exit (checkpoints already saved to JSON)
"""

import json
import math
import os
import sys
import threading

import rclpy
from rclpy.node import Node
from rclpy.time import Time

import tf2_ros
from geometry_msgs.msg import TransformStamped


CHECKPOINTS_FILE = os.path.join(
    os.path.dirname(__file__),   # slam_node/slam_node/
    '..', 'checkpoints', 'checkpoints.json'
)
CHECKPOINTS_FILE = os.path.normpath(CHECKPOINTS_FILE)

# Installed share-dir path (used at runtime after colcon build)
try:
    from ament_index_python.packages import get_package_share_directory
    _share = get_package_share_directory('slam_node')
    CHECKPOINTS_FILE_RUNTIME = os.path.join(_share, 'checkpoints', 'checkpoints.json')
except Exception:
    CHECKPOINTS_FILE_RUNTIME = CHECKPOINTS_FILE


def quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Extract yaw (rotation around Z) from a quaternion."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class CheckpointSaver(Node):
    """ROS 2 node that reads map→base_link TF and records named checkpoints."""

    def __init__(self) -> None:
        super().__init__('checkpoint_saver')

        # TF listener — reads the full TF tree
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # Load existing checkpoints from disk
        self._checkpoints: dict = self._load_checkpoints()

        self.get_logger().info(
            f'checkpoint_saver ready  |  file: {self._checkpoint_path()}'
        )

    # ── TF lookup ────────────────────────────────────────────────────────────

    def get_map_pose(self) -> dict | None:
        """
        Look up the transform  map → base_link  and return a pose dict.
        Returns None if the TF is not yet available (e.g. SLAM not started).
        """
        try:
            tf: TransformStamped = self._tf_buffer.lookup_transform(
                target_frame='map',
                source_frame='base_link',
                time=Time(),           # latest available
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as exc:
            self.get_logger().warning(f'TF lookup failed: {exc}')
            return None

        t = tf.transform.translation
        r = tf.transform.rotation
        yaw = quat_to_yaw(r.x, r.y, r.z, r.w)

        return {
            'x':   round(t.x,   4),
            'y':   round(t.y,   4),
            'yaw': round(yaw,   4),   # radians
        }

    # ── Persistence helpers ──────────────────────────────────────────────────

    def _checkpoint_path(self) -> str:
        # Prefer the runtime install path if it exists
        if os.path.isfile(CHECKPOINTS_FILE_RUNTIME):
            return CHECKPOINTS_FILE_RUNTIME
        # Fall back to source-tree path (dev workflow)
        os.makedirs(os.path.dirname(CHECKPOINTS_FILE), exist_ok=True)
        return CHECKPOINTS_FILE

    def _load_checkpoints(self) -> dict:
        path = self._checkpoint_path()
        if os.path.isfile(path):
            try:
                with open(path) as f:
                    data = json.load(f)
                self.get_logger().info(
                    f'Loaded {len(data)} checkpoint(s) from {path}'
                )
                return data
            except json.JSONDecodeError as exc:
                self.get_logger().error(f'Corrupt checkpoints file: {exc}')
        return {}

    def _save_checkpoints(self) -> None:
        path = self._checkpoint_path()
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, 'w') as f:
            json.dump(self._checkpoints, f, indent=2)
        self.get_logger().info(f'Checkpoints saved → {path}')

    # ── CLI actions ──────────────────────────────────────────────────────────

    def cmd_save(self) -> None:
        pose = self.get_map_pose()
        if pose is None:
            print('\n  ✗  Cannot read map→base_link TF.')
            print('     Is SLAM Toolbox running?  Is the robot localised?')
            return

        print(f'\n  Current pose (map frame):')
        print(f'    x = {pose["x"]:+.4f} m')
        print(f'    y = {pose["y"]:+.4f} m')
        print(f'    yaw = {pose["yaw"]:+.4f} rad  ({math.degrees(pose["yaw"]):+.1f}°)')

        name = input('\n  Checkpoint name (e.g. "home", "dock", "room_a"): ').strip()
        if not name:
            print('  ✗  Empty name — cancelled.')
            return

        if name in self._checkpoints:
            overwrite = input(f'  "{name}" already exists. Overwrite? [y/N]: ').strip().lower()
            if overwrite != 'y':
                print('  Cancelled.')
                return

        self._checkpoints[name] = pose
        self._save_checkpoints()
        print(f'  ✓  Checkpoint "{name}" saved.')

    def cmd_list(self) -> None:
        if not self._checkpoints:
            print('\n  No checkpoints saved yet.')
            return
        print(f'\n  {"NAME":<20}  {"X (m)":>8}  {"Y (m)":>8}  {"YAW (°)":>8}')
        print('  ' + '-' * 52)
        for name, p in sorted(self._checkpoints.items()):
            yaw_deg = math.degrees(p.get('yaw', 0.0))
            print(f'  {name:<20}  {p["x"]:>8.4f}  {p["y"]:>8.4f}  {yaw_deg:>8.1f}')

    def cmd_delete(self) -> None:
        self.cmd_list()
        if not self._checkpoints:
            return
        name = input('\n  Name to delete: ').strip()
        if name not in self._checkpoints:
            print(f'  ✗  "{name}" not found.')
            return
        del self._checkpoints[name]
        self._save_checkpoints()
        print(f'  ✓  Deleted "{name}".')

    # ── Interactive loop (runs in its own thread) ────────────────────────────

    def run_cli(self) -> None:
        banner = """
╔═══════════════════════════════════════════════════════╗
║          SLAM Checkpoint Saver  (map frame)           ║
╠═══════════════════════════════════════════════════════╣
║  s  —  Save current pose as a named checkpoint        ║
║  l  —  List all saved checkpoints                     ║
║  d  —  Delete a checkpoint                            ║
║  q  —  Quit                                           ║
╚═══════════════════════════════════════════════════════╝
"""
        print(banner)

        while rclpy.ok():
            try:
                cmd = input('checkpoint_saver> ').strip().lower()
            except (EOFError, KeyboardInterrupt):
                print('\nExiting…')
                break

            if cmd in ('s', 'save'):
                self.cmd_save()
            elif cmd in ('l', 'list'):
                self.cmd_list()
            elif cmd in ('d', 'del', 'delete'):
                self.cmd_delete()
            elif cmd in ('q', 'quit', 'exit'):
                print('Goodbye.')
                break
            elif cmd == '':
                pass
            else:
                print(f'  Unknown command "{cmd}".  Use s / l / d / q.')

        rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CheckpointSaver()

    # Spin ROS in a background thread so the CLI can block on input()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # CLI blocks the main thread
    node.run_cli()

    node.destroy_node()


if __name__ == '__main__':
    main()
