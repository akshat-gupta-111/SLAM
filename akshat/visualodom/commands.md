
1. Install the cuVSLAM Package
   Assuming you already have ROS 2 Humble installed, install the Isaac ROS visual SLAM binaries:

Bash
source /opt/ros/humble/setup.bash
sudo apt-get update
sudo apt-get install -y ros-humble-isaac-ros-visual-slam ros-humble-isaac-ros-visual-slam-interfaces


2. Create the Workspace and Launch Directory
   Bash
   mkdir -p ~/vo_ws/src/mono_slam_config/launch
   cd ~/vo_ws/src/mono_slam_config/launch



3. Create the Custom Launch File
   Run this command to automatically generate the Python launch file that forces monocular mode. It will write the code directly into a file named mono_cuvslam.launch.py:


cat << 'EOF' > mono_cuvslam.launch.py
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::cuvslam::VisualSlamNode',
        parameters=[{
            'enable_rectified_pose': False,
            'denoise_input_images': True,
            'rectified_images': True,
            'enable_slam_pose': False,
            'enable_image_topic_right': False,
            'publish_odom_to_base_tf': False
        }],
        remappings=[
            ('visual_slam/image_0', '/camera/image_raw'),
            ('visual_slam/camera_info_0', '/camera/camera_info'),
            ('visual_slam/tracking/odometry', '/odom_camera')
        ]
    )

    container = ComposableNodeContainer(
        name='cuvslam_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[visual_slam_node],
        output='screen'
    )

    return LaunchDescription([container])
EOF





4. Run the Camera Driver (Terminal 1)
   You must have your camera publishing first. If using a standard USB/Pi camera, open a terminal and run:

Bash
source /opt/ros/humble/setup.bash
sudo apt-get install -y ros-humble-usb-cam
ros2 run usb_cam usb_cam_node_exe




5. Launch the cuVSLAM Node (Terminal 2)
   Open a second terminal, source ROS 2, and launch the file you just created:

Bash
source /opt/ros/humble/setup.bash
ros2 launch ~/vo_ws/src/mono_slam_config/launch/mono_cuvslam.launch.py



6. Verify the EKF Parameters (Terminal 3)
   Open a third terminal and run this to print the visual odometry velocity parameters in real-time:

Bash
source /opt/ros/humble/setup.bash
ros2 topic echo /odom_camera


Once you run that final command, wave your camera around. You will see the twist.twist.linear.x and twist.twist.angular.z values updating on the screen—ready to be consumed by your EKF.
