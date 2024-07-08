#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # RViZ2 settings
    rviz2_config = os.path.join(
        get_package_share_directory("lidar"), "rviz2", "scan.rviz"
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_show",
        arguments=["-d", rviz2_config],
        output="screen",
    )
    # base_link to laser_frame tf node
    base_link_to_laser_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_base_laser_link",
        arguments=["0", "0", "0.18", "0", "0", "0", "base_link", "lidar_link"],
    )

    # Define LaunchDescription variable
    ord = LaunchDescription()

    ord.add_action(base_link_to_laser_tf_node)
    ord.add_action(rviz2_node)

    return ord
