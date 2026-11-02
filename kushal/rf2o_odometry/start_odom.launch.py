import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    # Automatically detect the folder where this launch file is kept
    script_dir = os.path.dirname(os.path.realpath(__file__))

    return LaunchDescription([
        # 1. RPLiDAR Launch (Only show warnings/errors)
        ExecuteProcess(
            cmd=['ros2', 'launch', 'rplidar_ros', 'rplidar_a1_launch.py', '--ros-args', '--log-level', 'WARN'],
            output='screen'
        ),
        
        # 2. Zameen se Base Link ki TF (0.035m upar)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0.035', '0', '0', '0', 'base_footprint', 'base_link', '--ros-args', '--log-level', 'WARN']
        ),

        # 3. Base Link se Laser ki TF (0.625m upar)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0.625', '0', '0', '0', 'base_link', 'laser', '--ros-args', '--log-level', 'WARN']
        ),

        # 4. C++ RF2O Node (Silent unless error)
        Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            parameters=[{
                'laser_scan_topic': '/scan',
                'odom_topic': '/odom_rf2o',
                'publish_tf': False,
                'base_frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'init_pose_from_topic': '',
                'freq': 10.0
            }],
            arguments=['--ros-args', '--log-level', 'WARN']
        ),

        # 5. EKF Data Collector Server (Main terminal output)
        ExecuteProcess(
            cmd=['python3', 'ekf_server.py'],
            cwd=script_dir,
            output='screen'
        ),

        # 6. RF2O Holonomic Python Worker (Silent unless error)
        ExecuteProcess(
            cmd=['python3', 'rf2o_worker.py'],
            cwd=script_dir,
            output='screen'
        )
    ])