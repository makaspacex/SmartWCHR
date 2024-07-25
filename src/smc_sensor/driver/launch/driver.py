import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, LaunchConfigurationEquals


def generate_launch_description():
    launch_ros_local = LaunchConfiguration("launch_ros_local", default="true")
    
    return launch.LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_name",
                default_value="gk01",
                choices=["gk01", "v1"],
                description="robot_name, gk01/v1",
            ),
            DeclareLaunchArgument(
                "launch_ros_local",
                default_value="true",
                description="start robot localization",
            ),
            Node(
                package="driver",
                executable="controller",
                name="driver_control",
                output="screen",
                condition=LaunchConfigurationEquals("robot_name", "v1"),
            ),
            Node(
                package="driver",
                executable="gk_controller",
                name="gk_controller",
                output="screen",
                condition=LaunchConfigurationEquals("robot_name", "gk01"),
            ),
            Node(
                package="driver",
                executable="odom_cal",
                name="odom_cal_node",
                output="screen",
                parameters=[{'robot_name': LaunchConfiguration('robot_name')}]
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("ros_local"),
                        "launch",
                        "localization.py",
                    )
                ),
                condition=IfCondition(launch_ros_local)
            )
        ]
    )
