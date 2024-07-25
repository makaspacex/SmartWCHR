#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from pathlib import Path


def generate_launch_description():
    package_name = Path(__file__).parent.parent.stem
    package_share_dir = get_package_share_directory(package_name)

    channel_type = LaunchConfiguration("channel_type", default="serial")
    serial_port = LaunchConfiguration("serial_port", default="/dev/rplidar")
    serial_baudrate = LaunchConfiguration(
        "serial_baudrate", default="1000000"
    )  # for s2 is 1000000
    frame_id = LaunchConfiguration("frame_id", default="laser_link")
    inverted = LaunchConfiguration("inverted", default="false")
    show_robot = LaunchConfiguration("show_robot", default="false")
    angle_compensate = LaunchConfiguration("angle_compensate", default="true")
    scan_mode = LaunchConfiguration("scan_mode", default="DenseBoost")

    rviz_config_dir = os.path.join(package_share_dir, "rviz", "sllidar.rviz")

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
                default_value="true",
                description="Specifying whether or not to enable angle_compensate of scan data",
            ),
            DeclareLaunchArgument(
                "scan_mode",
                default_value=scan_mode,
                description="Specifying scan mode of lidar",
            ),
            DeclareLaunchArgument(
                "show_robot",
                default_value=show_robot,
                description="If show robot",
            ),
            DeclareLaunchArgument(
                "start_angle",
                default_value="180",
                description="Start angle for the laser filter",
            ),
            DeclareLaunchArgument(
                "end_angle",
                default_value="70",
                description="End angle for the laser filter",
            ),
            DeclareLaunchArgument(
                "sub_topic",
                default_value="scan_s2_raw",
                description="subscription topic",
            ),
            DeclareLaunchArgument(
                "pub_topic",
                default_value="scan_s2_filter",
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
                        get_package_share_directory("wcmodel"), "launch", "wc_base.py"
                    )
                ),
                condition=IfCondition(show_robot),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config_dir],
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([package_share_dir, "/launch/scan.py"]),
                launch_arguments={
                    "channel_type": channel_type,
                    "serial_port": serial_port,
                    "serial_baudrate": serial_baudrate,
                    "frame_id": frame_id,
                    "inverted": inverted,
                    "angle_compensate": angle_compensate,
                    "scan_mode": scan_mode,
                    "start_angle": LaunchConfiguration("start_angle"),
                    "end_angle": LaunchConfiguration("end_angle"),
                    "sub_topic": LaunchConfiguration("sub_topic"),
                    "pub_topic": LaunchConfiguration("pub_topic"),
                }.items(),
                condition=IfCondition(LaunchConfiguration("use_fileter")),
            ),
        ]
    )
