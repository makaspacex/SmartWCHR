import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    mode = LaunchConfiguration('mode', default="nav2")
    
    return LaunchDescription(
        [
           DeclareLaunchArgument(
                "mode",
                default_value="nav2",
                choices=["nav2", "vis"],
                description="following mode, nav2/vis",
            ),
            Node(
            package='yolomix',
            executable='yolomix',
            name='yolomix_node',
            output='screen'

        ),
        Node(
            package='following_controller',
            executable='nav2',
            name='following_nav2',
            output='screen',
            condition = LaunchConfigurationEquals('mode', 'nav2'),
        ),

        Node(
            package='following_controller',
            executable='vis',
            name='following_vis',
            output='screen',
            condition = LaunchConfigurationEquals('mode', 'vis'),
        )
            
        ]
    )
