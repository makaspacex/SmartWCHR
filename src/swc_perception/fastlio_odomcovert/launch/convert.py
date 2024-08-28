import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition

from launch_ros.actions import Node
from pathlib import Path


def generate_launch_description():
    package_name = Path(__file__).parent.parent.stem
    package_share_dir = get_package_share_directory(package_name)
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    
    convert_node = Node(
        package='fastlio_odomcovert',
        executable='covert_node',
        name="fastlio_odomcovert",
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[("/covert_odomtry", "/odom")],
        output='screen'
    )
    ld = LaunchDescription()
    ld.add_action(convert_node)
    return ld
