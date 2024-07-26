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
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "show_robot",
                default_value="true",
                description="If show robot",
            ),
            DeclareLaunchArgument(
                "start_angle",
                default_value="0",
                description="Start angle for the laser filter",
            ),
            DeclareLaunchArgument(
                "end_angle",
                default_value="200",
                description="End angle for the laser filter",
            ),
            
            DeclareLaunchArgument(
                "sub_topic",
                default_value="scan_ms200_raw",
                description="subscription topic",
            ),
            DeclareLaunchArgument(
                "pub_topic",
                default_value="scan_ms200_filter",
                description="publisher topic",
            ),
            DeclareLaunchArgument(
                "use_fileter",
                default_value="true",
                description="if use fileter",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("swc_nav_bringup"), "launch", "wc_base.py"
                    )
                ),
                condition=IfCondition(LaunchConfiguration("show_robot", default="true")),
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
                ),
                launch_arguments={
                    "start_angle": LaunchConfiguration("start_angle"),
                    "end_angle": LaunchConfiguration("end_angle"),
                    "sub_topic": LaunchConfiguration("sub_topic"),
                    "pub_topic": LaunchConfiguration("pub_topic"),
                  }.items(),
                condition=IfCondition(LaunchConfiguration("use_fileter")),
            ),
        ]
    )
