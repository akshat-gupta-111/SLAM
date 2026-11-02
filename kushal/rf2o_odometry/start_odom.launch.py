import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    script_dir = os.path.dirname(os.path.realpath(__file__))

    # 1. RPLiDAR A1 Node (Using the actual node executable instead of nested launch to prevent argument errors)
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200,
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
        }]
    )

    # 2. Base Footprint to Base Link TF (0.035m upar)
    tf_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0.035', '0', '0', '0', 'base_footprint', 'base_link']
    )

    # 3. Base Link to Laser TF (0.625m upar)
    tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0.625', '0', '0', '0', 'base_link', 'laser']
    )

    # 4. C++ RF2O Node (Delayed by 3 seconds to let RPLiDAR initialize the motor and spin up /scan topic)
    rf2o_node = Node(
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
        }]
    )

    delayed_rf2o = TimerAction(
        period=3.0,
        actions=[rf2o_node]
    )

    # 5. EKF Data Collector Server (Delayed by 3.5 seconds)
    ekf_server_process = ExecuteProcess(
        cmd=['python3', 'ekf_server.py'],
        cwd=script_dir,
        output='screen'
    )

    delayed_ekf = TimerAction(
        period=3.5,
        actions=[ekf_server_process]
    )

    # 6. RF2O Holonomic Python Worker (Delayed by 3.5 seconds)
    rf2o_worker_process = ExecuteProcess(
        cmd=['python3', 'rf2o_worker.py'],
        cwd=script_dir,
        output='screen'
    )

    delayed_worker = TimerAction(
        period=3.5,
        actions=[rf2o_worker_process]
    )

    return LaunchDescription([
        rplidar_node,
        tf_footprint,
        tf_laser,
        delayed_rf2o,
        delayed_ekf,
        delayed_worker
    ])
