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
import math

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
            # Node(
            #     package="laser_filters",
            #     executable="scan_to_scan_filter_chain",
            #     name='laser_ms200_filter',
            #     output='screen',
            #     parameters=[
            #         {  
            #             "filter1": {
            #                 "name": "angle",
            #                 "type": "laser_filters/LaserScanAngularBoundsFilter",
            #                 "params": {
            #                     "lower_angle": 0/180 * math.pi, 
            #                     "upper_angle": 180/180*math.pi
            #                 },
            #             },
            #         }
            #     ],
            #     remappings=[("/scan",LaunchConfiguration("sub_topic")),("/scan_filtered",LaunchConfiguration("pub_topic"))]
            # ),

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
                    "start_angle": "0",
                    "end_angle": "200",
                    "sub_topic": "scan_ms200_raw",
                    "pub_topic": "scan_ms200_filter",
                  }.items(),
                condition=IfCondition(LaunchConfiguration("use_fileter")),
            )
        ]
    )

    return ord
