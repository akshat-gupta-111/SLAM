#!/usr/bin/env python3
import os
import json
import math
import time
import threading
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger


def euler_from_quaternion(x, y, z, w):
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t3, t4)


def quaternion_from_yaw(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class CheckpointControlNode(Node):
    """
    All-in-One Interactive Control Panel for:
    - Starting / Pausing Room Mapping
    - Saving .pgm and .yaml Map
    - Saving Named Checkpoints from live /odometry/filtered
    - Sending Robot to any Checkpoint or Patrolling all Checkpoints
    """

    def __init__(self):
        super().__init__('checkpoint_cli')
        self.declare_parameter('checkpoints_file', '~/Desktop/SLAM/slam_node/config/checkpoints.json')
        self.cp_file = os.path.expanduser(self.get_parameter('checkpoints_file').value)
        os.makedirs(os.path.dirname(self.cp_file), exist_ok=True)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.checkpoints = self.load_checkpoints()

        self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odom_cb, 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/nav/goal', 10)

        self.cli_start_map = self.create_client(Trigger, '/start_mapping')
        self.cli_stop_map = self.create_client(Trigger, '/stop_mapping')
        self.cli_save_map = self.create_client(Trigger, '/save_map')
        self.cli_cancel_nav = self.create_client(Trigger, '/nav/cancel')

    def odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)

    def load_checkpoints(self):
        if os.path.exists(self.cp_file):
            try:
                with open(self.cp_file, 'r') as f:
                    return json.load(f)
            except Exception:
                pass
        return {"home": {"x": 0.0, "y": 0.0, "yaw": 0.0}}

    def save_checkpoints(self):
        with open(self.cp_file, 'w') as f:
            json.dump(self.checkpoints, f, indent=2)

    def call_trigger(self, client, label):
        if not client.wait_for_service(timeout_sec=1.5):
            print(f"[!] Service '{label}' abhi chal nahi rahi hai. Check karein ki node active hai.")
            return
        req = Trigger.Request()
        future = client.call_async(req)
        time.sleep(0.4)
        if future.done() and future.result() is not None:
            print(f"[OK] {future.result().message}")
        else:
            print(f"[OK] Command '{label}' sent!")

    def send_goal(self, name, data):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(data['x'])
        msg.pose.position.y = float(data['y'])
        qx, qy, qz, qw = quaternion_from_yaw(float(data.get('yaw', 0.0)))
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self.goal_pub.publish(msg)
        print(f"-> Robot ko Checkpoint '{name}' ({data['x']:.2f}, {data['y']:.2f}) par bhej diya gaya hai!")


def interactive_menu(node: CheckpointControlNode):
    while rclpy.ok():
        print("\n=======================================================")
        print("   SLAM & CHECKPOINT CONTROLLER (Daksh / ROS 2)")
        print("=======================================================")
        print(f" Live Robot Position (/odometry/filtered): X={node.x:.2f}m, Y={node.y:.2f}m, Yaw={math.degrees(node.yaw):.1f} deg")
        print("-------------------------------------------------------")
        print(" [1] Start Room Mapping (/map banana shuru karein)")
        print(" [2] Pause Room Mapping")
        print(" [3] Save Map (.pgm & .yaml file save karein)")
        print(" [4] Mark Current Position as Checkpoint (Naya Checkpoint banayein)")
        print(" [5] List All Saved Checkpoints")
        print(" [6] Navigate to a Checkpoint (Robot ko Checkpoint pe bhejein)")
        print(" [7] Patrol All Checkpoints One-by-One")
        print(" [8] STOP Robot / Cancel Navigation")
        print(" [0] Exit")
        print("=======================================================")

        choice = input("Apna option chunein (0-8): ").strip()

        if choice == '1':
            node.call_trigger(node.cli_start_map, '/start_mapping')
        elif choice == '2':
            node.call_trigger(node.cli_stop_map, '/stop_mapping')
        elif choice == '3':
            node.call_trigger(node.cli_save_map, '/save_map')
        elif choice == '4':
            cp_name = input("Checkpoint ka naam likhein (jaise cp1, gate, table): ").strip()
            if cp_name:
                node.checkpoints[cp_name] = {
                    "x": round(node.x, 3),
                    "y": round(node.y, 3),
                    "yaw": round(node.yaw, 3)
                }
                node.save_checkpoints()
                print(f"[SAVED] Checkpoint '{cp_name}' save ho gaya: {node.checkpoints[cp_name]}")
        elif choice == '5':
            print("\n--- Saved Checkpoints ---")
            for idx, (k, v) in enumerate(node.checkpoints.items(), 1):
                print(f"  {idx}. {k} -> X: {v['x']}m, Y: {v['y']}m, Yaw: {v['yaw']} rad")
        elif choice == '6':
            names = list(node.checkpoints.keys())
            for idx, k in enumerate(names, 1):
                v = node.checkpoints[k]
                print(f"  [{idx}] {k} ({v['x']}m, {v['y']}m)")
            sel = input("Checkpoint ka naam ya number likhein: ").strip()
            if sel.isdigit() and 1 <= int(sel) <= len(names):
                sel_name = names[int(sel) - 1]
                node.send_goal(sel_name, node.checkpoints[sel_name])
            elif sel in node.checkpoints:
                node.send_goal(sel, node.checkpoints[sel])
            else:
                print("[!] Invalid checkpoint selection.")
        elif choice == '7':
            print("[PATROL] Sabhi checkpoints par ek-ek karke ja raha hai...")
            for k, v in node.checkpoints.items():
                node.send_goal(k, v)
                while True:
                    dist = math.hypot(v['x'] - node.x, v['y'] - node.y)
                    if dist < 0.25:
                        print(f"[REACHED] '{k}' par pahunch gaya! Agle checkpoint par ja raha hai...")
                        time.sleep(1.5)
                        break
                    time.sleep(0.5)
        elif choice == '8':
            node.call_trigger(node.cli_cancel_nav, '/nav/cancel')
        elif choice == '0':
            print("Exiting controller...")
            break


def main(args=None):
    rclpy.init(args=args)
    node = CheckpointControlNode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    try:
        interactive_menu(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
