from launch import LaunchDescription
from launch_ros.actions import Node 
import os
from launch.actions import IncludeLaunchDescription
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource,AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
import os

def generate_launch_description():
	LIDAR_TYPE = os.getenv('LIDAR_TYPE')
	print("my_lidar:",LIDAR_TYPE)
	lidar_type_arg = DeclareLaunchArgument(name='lidar_type', default_value=LIDAR_TYPE, 
                                              description='The type of lidar')
	lidar_x3_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('ydlidar_ros2_driver'), 'launch'),
        '/x3_ydlidar_launch.py']),
         condition=LaunchConfigurationEquals('lidar_type', 'x3')
    )
	lidar_4ros_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('ydlidar_ros2_driver'), 'launch'),
        '/4ros_ydlidar_launch.py']),
         condition=LaunchConfigurationEquals('lidar_type', '4ros')
    )
	
	return LaunchDescription([
        lidar_type_arg,
        lidar_x3_launch,
        lidar_4ros_launch
    ])

