from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rtabmap_ros',
            executable='rgbd_odometry',
            name='visual_odometry',
            output='screen',
            parameters=['~/ros2_ws/src/robot_vo_bringup/config/vo_params.yaml'],
            remappings=[
                ('rgb/image', '/camera/image_raw'),
                ('rgb/camera_info', '/camera/camera_info'),
                ('odom', '/vo/odom')
            ]
        )
    ])