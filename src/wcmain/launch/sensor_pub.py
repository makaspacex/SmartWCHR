import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument
from pathlib import Path


def generate_launch_description():
    package_name = Path(__file__).parent.parent.stem
    package_share_dir = get_package_share_directory(package_name)

    lidar = LaunchConfiguration("lidar")
    declare_lidar_type_cmd = DeclareLaunchArgument(
        "lidar",
        default_value="lidar",
        description="lidar pacakge name, lidar/sllidar/lslidar_ros",
    )
    return LaunchDescription(
        [
            declare_lidar_type_cmd,
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("driver"), "launch", "driver.py"
                    )
                )
            ),
            # 条件启动ms200雷达
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("lidar"), "launch", "scan.py"
                    )
                ),
                condition=IfCondition(PythonExpression(["'", lidar, "' == 'lidar'"])),
            ),
            # 条件启动思岚s2雷达
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("sllidar"), "launch", "scan.py"
                    )
                ),
                condition=IfCondition(PythonExpression(["'", lidar, "' == 'sllidar'"])),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory("imu"), "launch", "imu.py")
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("wcmodel"), "launch", "wc_base.py"
                    )
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("wcmain"),
                        "launch",
                        "particle_to_point.py",
                    )
                )
            ),
        ]
    )
