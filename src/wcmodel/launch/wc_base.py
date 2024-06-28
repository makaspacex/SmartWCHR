import os

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = 'wcmodel'
    urdf_name = "wheelchair_base.urdf"

    ld = LaunchDescription()
    # pkg_share = FindPackageShare(package=package_name).find(package_name) 
    # urdf_model_path = os.path.join(pkg_share, f'urdf/{urdf_name}')

    # current_file_path = os.path.dirname(os.path.abspath(__file__))
    # urdf_directory = os.path.join(current_file_path, '..', '..', 'urdf')
    # urdf_directory = os.path.normpath(urdf_directory)
    # urdf_model_path = os.path.join(urdf_directory, urdf_name)

    pkg_share = FindPackageShare(package=package_name).find(package_name)
    urdf_model_path = os.path.join(pkg_share, f'urdf/{urdf_name}')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[urdf_model_path]
        )

    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        arguments=[urdf_model_path]
        )


    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    #ld.add_action(rviz2_node)

    return ld
