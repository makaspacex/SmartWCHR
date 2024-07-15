import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_name = 'nav2_sample'
    urdf_name = "wheelchair_base.urdf"
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    urdf_model_path = os.path.join(pkg_share, f'urdf/{urdf_name}')
    urdf_model_path = "/home/makafly/Desktop/ResearchNav2/src/nav2_sample/urdf/wheelchair_base.urdf"
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    robot_description = Command([
        FindExecutable(name='xacro'),
        ' ',  # xacro和文件路径之间的空格
        urdf_model_path
    ])
    parameters=[{'use_sim_time':use_sim_time, 'robot_description': robot_description, 'use_gui': False}]
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=parameters,
        )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_gui': False, 'use_sim_time':use_sim_time,}],
        )
    
    ld = LaunchDescription()
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)

    return ld
