"""Launch the EKF with its configuration and the two calibration files."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("ekf_node")
    cfg = os.path.join(share, "config")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            package="ekf_node",
            executable="ekf_localization_node",
            name="ekf_localization_node",   # must match the YAML's top-level key
            output="screen",
            emulate_tty=True,
            additional_env={
                "OPENBLAS_NUM_THREADS": "1",
                "OMP_NUM_THREADS": "1",
                "MKL_NUM_THREADS": "1",
            },
            parameters=[
                os.path.join(cfg, "ekf_localization.yaml"),
                {   # later entries override the YAML
                    "imu.calibration_file": os.path.join(cfg, "imu_calibration.yaml"),
                    "rf2o.calibration_file": os.path.join(cfg, "rf2o_calibration.yaml"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                },
            ],
        ),
    ])
