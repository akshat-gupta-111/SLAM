"""
mapping.launch.py — Industry-standard SLAM Toolbox mapping launch

What this starts:
  • slam_toolbox/async_slam_toolbox_node  — reads /scan + TF odom→base_link,
                                            publishes /map and TF map→odom

What this does NOT start (run separately before this):
  • IMU driver              (Terminal 1)
  • rf2o_laser_odometry     (Terminal 2)
  • robot_localization EKF  (Terminal 2, same launch as rf2o)

TF tree during mapping:
  map ──(slam_toolbox)──▶ odom ──(EKF)──▶ base_link
                                              ├──▶ imu_link
                                              └──▶ laser
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('slam_node')

    # ── Launch arguments ────────────────────────────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock (set true only in Gazebo)')

    params_file_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(pkg_share, 'config', 'slam_toolbox_mapping.yaml'),
        description='Full path to SLAM Toolbox parameter file')

    use_sim_time  = LaunchConfiguration('use_sim_time')
    params_file   = LaunchConfiguration('slam_params_file')

    # ── SLAM Toolbox (async mode — lighter on Jetson CPU) ───────────────────
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time},
        ],
        # Remap if your scan topic differs from /scan
        remappings=[
            ('/scan', '/scan'),
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        params_file_arg,
        LogInfo(msg='[mapping.launch] Starting SLAM Toolbox (async mode) …'),
        LogInfo(msg='[mapping.launch] Make sure EKF + rf2o are already running!'),
        slam_toolbox_node,
    ])
