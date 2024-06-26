import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # oradar_launch_file = "/home/jetson/Desktop/data/cyq/slam/slam_2d_ws/src/oradar_ros/launch/ms200_scan_view.launch.py"
    # imu_launch_file = "/home/jetson/Desktop/data/cyq/slam/slam_2d_ws/src/wit_ros2_imu/launch/rviz_and_imu.launch.py"

    oradar_launch_file = os.path.join(
        get_package_share_directory('oradar_lidar'),
        'launch',
        'ms200_scan.launch.py'
        # 'ms200_scan_view.launch.py'
    )
    imu_launch_file = os.path.join(
        get_package_share_directory('wit_ros2_imu'),
        'launch',
        'rviz_and_imu.launch.py'
    )

    description_launch_file = os.path.join(
        get_package_share_directory('wheelchair_description'),
        'launch',
        'wheelchair_description.launch.py'
    )

    slam_launch_file = os.path.join(
        get_package_share_directory('slam_2d'),
        'launch',
        'slam_2d.launch.py'
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
