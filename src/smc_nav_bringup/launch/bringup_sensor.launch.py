import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument
from pathlib import Path


def generate_launch_description():
    package_name = Path(__file__).parent.parent.stem
    package_share_dir = get_package_share_directory(package_name)

    robot_name = LaunchConfiguration('robot_name')
    launch_driver = LaunchConfiguration("launch_driver", default="true")
    launch_ros_local = LaunchConfiguration("launch_ros_local", default="true")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "lidar",
                default_value="sllidar",
                choices=["lidar_ms200", "sllidar", "lslidar_ros"],
                description="lidar pacakge name, lidar_ms200/sllidar/lslidar_ros",
            ),
            DeclareLaunchArgument(
                "robot_name",
                default_value="gk01",
                choices=["gk01", "v1"],
                description="robot_name, gk01/v1",
            ),
            DeclareLaunchArgument(
                "launch_driver",
                default_value="true",
                description="launch_driver",
            ),
            DeclareLaunchArgument(
                "launch_ros_local",
                default_value="true",
                description="start robot localization",
            ),
            # 手柄控制
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("joy_ctrl"), "launch", "joy_ctrl.py"
                    ),
                ),
                launch_arguments={"robot_name": robot_name,"launch_driver":launch_driver, "launch_ros_local":launch_ros_local}.items(),
            ),
            # 条件启动ms200雷达
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("lidar_ms200"), "launch", "scan.py"
                    )
                ),
                condition=LaunchConfigurationEquals("lidar", "lidar_ms200"),
            ),
            # 条件启动思岚s2雷达
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("sllidar"), "launch", "scan.py"
                    )
                ),
                condition=LaunchConfigurationEquals("lidar", "sllidar"),
            ),
            # 条件启动过滤器
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("radar_filter"), "launch", "laser_filter.launch.py"
                    )
                ),
                condition=LaunchConfigurationEquals("lidar", "sllidar"),
            ),
            # 启动imu
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory("imu"), "launch", "imu.py")
                )
            )
        ]
    )
