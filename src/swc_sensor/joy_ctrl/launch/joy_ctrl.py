import os
from pathlib import Path
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, LaunchConfigurationEquals


def generate_launch_description():
    package_name = Path(__file__).parent.parent.stem
    package_share_dir = get_package_share_directory(package_name)
    
    robot_name = LaunchConfiguration("robot_name")
    launch_driver = LaunchConfiguration("launch_driver", default="false")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_name",
                default_value="gk01",
                choices=["gk01", "v1"],
                description="robot_name, gk01/v1",
            ),
            DeclareLaunchArgument(
                "launch_driver",
                default_value="false",
                description="launch_driver",
            ),
            
            # 启动手柄控制
            Node(
                package=package_name,
                executable="wc_ctrl",
                output="screen",
            ),
            
            # 启动手柄服务
            Node(package="joy", executable="joy_node"),
            
            # 启动驱动器
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("driver"), "launch", "driver.py"
                    ),
                ),
                launch_arguments={"robot_name": robot_name}.items(),
                condition=IfCondition(launch_driver),
            ),
        ]
    )
