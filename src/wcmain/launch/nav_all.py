import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription(
        [
            IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("wcmain"), "launch", "sensor_pub.py"))),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("nav2"), "launch", "nav2.launch.py"))),
        ]
    )