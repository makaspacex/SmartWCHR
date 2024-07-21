import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    robot_name = LaunchConfiguration('robot_name')
    lidar = LaunchConfiguration('lidar')
    
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "lidar",
                default_value="sllidar",
                choices=["lidar", "sllidar", "lslidar_ros"],
                description="lidar pacakge name, lidar/sllidar/lslidar_ros",
            ),
            DeclareLaunchArgument(
                "robot_name",
                default_value="gk01",
                choices=["gk01", "v1"],
                description="lidar pacakge name, lidar/sllidar/lslidar_ros",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("wcmain"), "launch", "sensor_pub.py"
                    )
                ),
                launch_arguments={'robot_name': robot_name,"lidar":lidar}.items()
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("slam_2d"), "launch", "slam2d.py"
                    )
                )
            ),
        ]
    )
