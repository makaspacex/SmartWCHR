import launch_ros
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    driver_node = launch_ros.actions.Node(
        package="driver",
        executable="controller",
        name="driver_control",
        output="screen",
    )
    odom_msg_node = launch_ros.actions.Node(
        package="driver",
        executable="odom_cal",
        name="odom_msg",
        output="screen",
    )
    return launch.LaunchDescription(
        [
            driver_node,
            odom_msg_node,
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("ros_local"),
                        "launch",
                        "localization.py",
                    )
                )
            ),
        ]
    )
