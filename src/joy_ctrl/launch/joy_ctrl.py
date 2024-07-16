import os
from pathlib import Path
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_name = Path(__file__).parent.parent.stem
    package_share_dir = get_package_share_directory(package_name)

    return LaunchDescription(
        [
            Node(
                package=package_name,
                executable="wc_ctrl",
                output="screen",
            ),
            Node(package="joy", executable="joy_node"),
        ]
    )
