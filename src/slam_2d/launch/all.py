import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # oradar_launch_file = "/home/jetson/Desktop/data/cyq/slam/slam_2d_ws/src/lidar/launch/scan_view.py"
    # imu_launch_file = "/home/jetson/Desktop/data/cyq/slam/slam_2d_ws/src/imu/launch/imu.py"

    oradar_launch_file = os.path.join(
        get_package_share_directory('lidar'),
        'launch',
        'scan.py'
        # 'scan_view.py'
    )
    imu_launch_file = os.path.join(
        get_package_share_directory('imu'),
        'launch',
        'imu.py'
    )

    description_launch_file = os.path.join(
        get_package_share_directory('wcmodel'),
        'launch',
        'wc_base.py'
    )

    slam_launch_file = os.path.join(
        get_package_share_directory('slam_2d'),
        'launch',
        'slam_2d.py'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(oradar_launch_file)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(imu_launch_file)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch_file)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch_file)
        ),
    ])
