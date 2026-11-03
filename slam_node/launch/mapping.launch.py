import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('slam_node')
    params_file = os.path.join(pkg_share, 'config', 'slam_params.yaml')

    return LaunchDescription([
        Node(
            package='slam_node',
            executable='mapper_utility',
            name='mapper_utility',
            output='screen',
            parameters=[params_file]
        )
    ])
