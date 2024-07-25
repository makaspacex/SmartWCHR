#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
import os
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    package_name = Path(__file__).parent.parent.stem
    package_share_dir = get_package_share_directory(package_name)
    
    # Define LaunchDescription variable
    ord = LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_name",
                default_value="gk01",
                choices=["gk01", "v1"],
                description="robot_name, gk01/v1",
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
            Node(
                package=package_name,
                executable="scan",
                name="lidar",
                output="screen",
                parameters=[
                    {"device_model": "MS200"},
                    {"frame_id": "laser_ms200_link"},
                    {"scan_topic": "/scan_ms200_raw"},
                    {"port_name": "/dev/oradar"},
                    {"baudrate": 230400},
                    {"angle_min": 0.0},
                    {"angle_max": 360.0},
                    {"range_min": 0.1},
                    {"range_max": 10.0},
                    {"clockwise": False},
                    {"motor_speed": 10},
                ],
            ),
            # 启用过滤器
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("radar_filter"),
                        "launch",
                        "laser_filter.launch.py",
                    )
                ),
                launch_arguments={
                    "start_angle": LaunchConfiguration("start_angle"),
                    "end_angle": LaunchConfiguration("end_angle"),
                    "sub_topic": LaunchConfiguration("sub_topic"),
                    "pub_topic": LaunchConfiguration("pub_topic"),
                  }.items(),
                condition=IfCondition(LaunchConfiguration("use_fileter")),
            )
        ]
    )

    return ord
