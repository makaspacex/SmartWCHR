# #底层驱动
# ros2 run yahboomcar_bringup Mcnamu_driver_X3
# #手柄控制
# ros2 run joy_ctrl yahboom_joy_X3
# ros2 run joy joy_node

"""
This module provides slam function for wheelchair.

Functions:
- generate_launch_description(): launch nodes for slam_2d.
"""

import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='yahboomcar_bringup',
            executable='Mcnamu_driver_X3',
            name='Mcnamu_driver_X3'
        ),
        Node(
            package='joy_ctrl',
            executable='yahboom_joy_X3',
            name='yahboom_joy_X3'
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node'
        ),
    ])