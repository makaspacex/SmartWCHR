from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():

    # 是否使用仿真时间
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # 定位到 cartographer 功能包的地址
    pkg_share = FindPackageShare('cartographer_ros').find('cartographer_ros')
    configuration_directory = '/home/jetson/Desktop/data/cyq/slam/slam_2d_ws/src/slam_2d/config'
    configuration_basename = 'slam_2d.lua'
    pbstream_path = "/home/jetson/Desktop/data/cyq/slam/slam_2d_ws/src/slam_2d/map/map_test.pbstream"

    base_footprint_to_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_to_base_link',
        arguments=['0','0','-0.54','0','0','0','base_link','base_footprint']
    )

    base_link_to_imu_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_base_imu',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link']
    )    
    
    # base_link to laser_frame tf node
    base_link_to_laser_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_base_laser',
        arguments=['0','0','0.18','0','0','0','base_link','lidar']
    )


    cartographer_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_node',
        arguments = [
            '-configuration_directory', configuration_directory,
            # '-load_state_filename', pbstream_path,
            '-configuration_basename', configuration_basename],
        remappings = [
            ('scan', 'scan'),
            ('imu', 'imu/data_raw')
            # ('scan', 'scan')
            ],
        parameters=[{'use_sim_time': use_sim_time}],
        output = 'screen'
        )

    rviz_node = Node(
        package='rviz2',
        # namespace='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen')

    occupancy_grid_node = Node(
        package = 'cartographer_ros',
        executable = 'occupancy_grid_node',
        parameters = [
            {'use_sim_time': use_sim_time},
            {'resolution': 0.05}],
        )

    
    ld = LaunchDescription()
    ld.add_action(base_footprint_to_base_link)
    ld.add_action(base_link_to_imu_tf_node) #------
    ld.add_action(base_link_to_laser_tf_node)

    ld.add_action(rviz_node)
    ld.add_action(cartographer_node)
    ld.add_action(occupancy_grid_node)  
    
    # return LaunchDescription([
    #     use_sim_time_arg,
    #     rviz_node,
    #     cartographer_node,
    #     cartographer_occupancy_grid_node,
    # ])
    return ld

