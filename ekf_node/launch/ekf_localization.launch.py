import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(get_package_share_directory('ekf_node'), 'config', 'ekf_localization.yaml')
    return LaunchDescription([
        Node(package='ekf_node', executable='ekf_localization_node', parameters=[config])
    ])
