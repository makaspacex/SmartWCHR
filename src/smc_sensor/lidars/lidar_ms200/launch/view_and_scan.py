#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from pathlib import Path

def generate_launch_description():
    package_name = Path(__file__).parent.parent.stem
    package_share_dir = get_package_share_directory(package_name)
    
    # RViZ2 settings
    rviz2_config = os.path.join(package_share_dir, "rviz2", "scan.rviz")
    show_robot = LaunchConfiguration("show_robot", default="false")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "show_robot",
                default_value=show_robot,
                description="If show robot",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("wcmodel"), "launch", "wc_base.py"
                    )
                ),
                condition=IfCondition(show_robot),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2_show",
                arguments=["-d", rviz2_config],
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([package_share_dir, "/launch/scan.py"]
                )
            ),
        ]
    )
