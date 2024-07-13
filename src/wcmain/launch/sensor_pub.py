import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # description_launch_file = os.path.join(get_package_share_directory("wcmodel"), "launch", "wc_base.py")
    # slam_launch_file = os.path.join(get_package_share_directory("slam_2d"), "launch", "slam_2d.py")
    particle_to_point_node = Node(
        package='wcmain',
        executable='particle_to_point',
        name='particle_to_point',
        output='screen'
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("driver"), "launch", "driver.py"))),
            IncludeLaunchDescription(PythonLaunchDescriptionSource( os.path.join(get_package_share_directory("lidar"), "launch", "scan.py"))),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("imu"), "launch", "start.py"))),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("ros_local"), "launch", "localization.py"))),
            particle_to_point_node
        ]
    )