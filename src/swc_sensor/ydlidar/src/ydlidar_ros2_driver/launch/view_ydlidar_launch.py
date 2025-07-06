#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    rviz_config_dir = os.path.join(
            get_package_share_directory('ydlidar_ros2_driver'),
            'config',
            'ydlidar.rviz')


    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('ydlidar_ros2_driver'), 'launch'),
'/ydliar_launch.py'])
),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen'),
    ])


