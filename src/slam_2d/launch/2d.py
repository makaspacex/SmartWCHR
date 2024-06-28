import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():

    # do not use sim time
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # find config lua file
    slam_package_path = FindPackageShare('slam_2d').find('slam_2d')
    lua_name = 'slam_2d.lua'
    lua_directory = os.path.join(slam_package_path, 'config')
    
    # find wheelchair model launch file
    wcmodel_package_path = FindPackageShare('wcmodel').find('wcmodel')
    wheelchair_launch_file = os.path.join(wcmodel_package_path, 'launch', 'wc_base.py')
    
    # set rviz config file path
    rviz_config_path = str( Path(FindPackageShare('slam_2d').find('slam_2d')) / Path("config/rviz2.rviz"))
    
    # # launch the nodes below
    # return LaunchDescription([
    #     # coordinate transform nodes
    #     Node(
    #         package='tf2_ros',
    #         executable='static_transform_publisher',
    #         name='base_footprint_to_base_link',
    #         arguments=['0','0','-0.54','0','0','0','base_link','base_footprint']
    #     ),

    #     Node(
    #         package='tf2_ros',
    #         executable='static_transform_publisher',
    #         name='base_link_to_base_imu',
    #         arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link']
    #     ),

    #     Node(
    #         package='tf2_ros',
    #         executable='static_transform_publisher',
    #         name='base_link_to_base_laser',
    #         arguments=['0','0','0.18','0','0','0','base_link','lidar']
    #     ),

    #     # carto nodes
    #     Node(
    #         package = 'cartographer_ros',
    #         executable = 'cartographer_node',
    #         arguments = [
    #             '-configuration_directory', lua_directory,
    #             # '-load_state_filename', pbstream_path,
    #             '-configuration_basename', lua_name],
    #         remappings = [
    #             ('scan', 'scan'),
    #             ('imu', 'imu/data_raw')
    #             ],
    #         parameters=[{'use_sim_time': use_sim_time}],
    #         output = 'screen'
    #         ),

    #     Node(
    #         package = 'cartographer_ros',
    #         executable = 'cartographer_occupancy_grid_node', # use occupancy_grid_node instead in foxy
    #         parameters = [
    #             {'use_sim_time': use_sim_time},
    #             {'resolution': 0.05}],
    #         ),

    #     # visualization nodes
    #     IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(wheelchair_launch_file)
    #         ),

    #     Node(
    #         package='rviz2',
    #         executable='rviz2',
    #         name='rviz2',
    #         output='screen')
    # ])


    # tf nodes
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

    base_link_to_laser_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_base_laser',
        arguments=['0','0','0.18','0','0','0','base_link','lidar']
    )

    # carto nodes
    cartographer_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_node',
        arguments = [
            '-configuration_directory', lua_directory,
            # '-load_state_filename', pbstream_path,
            '-configuration_basename', lua_name],
        remappings = [
            ('scan', 'scan'),
            ('imu', 'imu/data_raw')
            ],
        parameters=[{'use_sim_time': use_sim_time}],
        output = 'screen'
        )
    
    occupancy_grid_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_occupancy_grid_node', # use occupancy_grid_node instead in foxy
        parameters = [
            {'use_sim_time': use_sim_time},
            {'resolution': 0.05}],
        )
    
    # visualization nodes
    wcmodel_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(wheelchair_launch_file)
        )

   
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen')

    # launch nodes above
    return LaunchDescription([
        base_footprint_to_base_link,
        base_link_to_imu_tf_node,
        base_link_to_laser_tf_node,
        wcmodel_node,
        rviz_node,
        cartographer_node,
        occupancy_grid_node
    ])
