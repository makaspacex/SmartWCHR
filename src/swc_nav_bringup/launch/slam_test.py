import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction, TimerAction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, TextSubstitution, PythonExpression
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals, IfCondition
from pathlib import Path

def generate_launch_description():
    package_name = Path(__file__).parent.parent.stem
    package_share_dir = get_package_share_directory(package_name)
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_toolbox_map_file =os.path.join(package_share_dir, 'map/FX1010_TEST.yaml') # TODO 这个地方无法拼接好了，先暂时这么使用
    slam_toolbox_localization_file = os.path.join(package_share_dir, 'config', 'reality', 'mapper_params_localization_real.yaml')

    start_localization_group = Node(
                condition = LaunchConfigurationEquals('localization', 'slam_toolbox'),
                package='slam_toolbox',
                executable='localization_slam_toolbox_node',
                name='localization_slam_toolbox_node',
                parameters=[
                    slam_toolbox_localization_file,
                    {'use_sim_time': use_sim_time,
                    'map_file_name': slam_toolbox_map_file,
                    'map_start_pose': [0.0, 0.0, 0.0]}
                ],
            )
    ld = LaunchDescription()
    ld.add_action(start_localization_group)
    print(slam_toolbox_map_file)
    return ld
