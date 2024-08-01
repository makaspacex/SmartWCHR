#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import Node
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
import math


def generate_launch_description():
    channel_type = LaunchConfiguration("channel_type", default="serial")
    serial_port = LaunchConfiguration("serial_port", default="/dev/rplidar")
    serial_baudrate = LaunchConfiguration(
        "serial_baudrate", default="1000000"
    )  # for s2 is 1000000
    frame_id = LaunchConfiguration("frame_id", default="laser_link")
    inverted = LaunchConfiguration("inverted", default="false")
    angle_compensate = LaunchConfiguration("angle_compensate", default="false")
    scan_mode = LaunchConfiguration("scan_mode", default="DenseBoost")
    use_sim_time = LaunchConfiguration('use_sim_time',default="false")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "channel_type",
                default_value=channel_type,
                description="Specifying channel type of lidar",
            ),
            DeclareLaunchArgument(
                "serial_port",
                default_value=serial_port,
                description="Specifying usb port to connected lidar",
            ),
            DeclareLaunchArgument(
                "serial_baudrate",
                default_value=serial_baudrate,
                description="Specifying usb port baudrate to connected lidar",
            ),
            DeclareLaunchArgument(
                "frame_id",
                default_value=frame_id,
                description="Specifying frame_id of lidar",
            ),
            DeclareLaunchArgument(
                "inverted",
                default_value=inverted,
                description="Specifying whether or not to invert scan data",
            ),
            DeclareLaunchArgument(
                "angle_compensate",
                default_value=angle_compensate,
                description="Specifying whether or not to enable angle_compensate of scan data",
            ),
            DeclareLaunchArgument(
                "scan_mode",
                default_value=scan_mode,
                description="Specifying scan mode of lidar",
            ),
            DeclareLaunchArgument(
                "start_angle",
                default_value="185",
                description="Start angle for the laser filter",
            ),
            DeclareLaunchArgument(
                "end_angle",
                default_value="70",
                description="End angle for the laser filter",
            ),
            DeclareLaunchArgument(
                "use_fileter",
                default_value="true",
                description="if use fileter",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="if use_sim_time",
            ),
            Node(
                package="sllidar",
                executable="sllidar_node",
                name="lidar_s2",
                parameters=[
                    {
                        "use_sim_time":use_sim_time,
                        "channel_type": channel_type,
                        "serial_port": serial_port,
                        "serial_baudrate": serial_baudrate,
                        "frame_id": frame_id,
                        "inverted": inverted,
                        "angle_compensate": angle_compensate,
                        "scan_mode": scan_mode,
                    }
                ],
                output="screen",
            ),
            # 启用过滤器
            # Node(
            #     package="laser_filters",
            #     executable="scan_to_scan_filter_chain",
            #     name="laser_s2_filter",
            #     output="screen",
            #     parameters=[
            #         {
            #             "filter1": {
            #                 "name": "angle",
            #                 "type": "laser_filters/LaserScanAngularBoundsFilter",
            #                 "params": {
            #                     "lower_angle": -80 / 180 * math.pi,
            #                     "upper_angle": 0 / 180 * math.pi,
            #                 },
            #             },
            #         }
            #     ],
            #     remappings=[
            #         ("/scan", "/scan_s2_raw"),
            #         ("/scan_filtered", "/scan_s2_filter"),
            #     ],
            # ),
            
            # 启用过滤器
            Node(
                package="radar_filter",
                executable="radar_filter",
                name="laser_filter_s2",
                parameters=[{
                    "use_sim_time":use_sim_time,
                    "start_angle": 210,
                    "end_angle": 70,
                    }],
                output="screen",
                 remappings=[
                    ("/scan_raw", "/scan_s2_raw"),
                    ("/scan_filter", "/scan_s2_filter"),
                ],
            )
        ]
    )
