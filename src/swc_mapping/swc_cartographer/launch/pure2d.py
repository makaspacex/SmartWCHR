"""
This module provides slam function for wheelchair.

Functions:
- generate_launch_description(): launch nodes for pure2d.
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
    """
    Function to build map using pure laser.
    """

    # do not use sim time
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # find config lua file
    slam_package_path = FindPackageShare('slam_2d').find('slam_2d')
    lua_name = 'pure2d.lua'
    lua_directory = os.path.join(slam_package_path, 'config')

    # find wheelchair model launch file
    swc_nav_bringup_package_path = FindPackageShare('swc_nav_bringup').find('swc_nav_bringup')
    wheelchair_launch_file = os.path.join(swc_nav_bringup_package_path, 'launch', 'wc_base.py')

    # set rviz config file path
    rviz_config_path = str( Path(slam_package_path ) / Path("config/pure2d.rviz"))

    # carto nodes
    cartographer_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_node',
        arguments = [
            '-configuration_directory', lua_directory,
            '-configuration_basename', lua_name],
        remappings = [
            ('scan', 'scan')
            ],
        parameters=[{'use_sim_time': use_sim_time}],
        output = 'screen'
        )

    occupancy_grid_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_occupancy_grid_node', # use occupancy_grid_node instead in foxy
        parameters = [
            {'use_sim_time': use_sim_time},
            {'resolution': 0.05}],
        )

    # visualization nodes
    swc_nav_bringup_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(wheelchair_launch_file)
        )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen')

    # launch nodes above
    return LaunchDescription([
        base_footprint_to_base_link,
        base_link_to_laser_tf_node,
        swc_nav_bringup_node,
        rviz_node,
        cartographer_node,
        occupancy_grid_node
    ])
