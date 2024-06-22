import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    amcl_yaml = os.path.join(get_package_share_directory('guide_nav2'), 'config', 'amcl.yaml')
    bt_navigator_yaml = os.path.join(get_package_share_directory('guide_nav2'), 'config', 'bt_navigator.yaml')
    controller_server_yaml = os.path.join(get_package_share_directory('guide_nav2'), 'config', 'controller_server.yaml')
    planner_server_yaml = os.path.join(get_package_share_directory('guide_nav2'), 'config', 'planner_server.yaml')
    recoveries_server_yaml = os.path.join(get_package_share_directory('guide_nav2'), 'config', 'recoveries_server.yaml')
    # acml_yaml = "/home/jetson/Desktop/data/cyq/slam/guide_nav2_ws/src/guide_nav2/config/acml.yaml"
    # bt_navigator_yaml = "/home/jetson/Desktop/data/cyq/slam/guide_nav2_ws/src/guide_nav2/config/bt_navigator.yaml"
    # controller_server_yaml = "/home/jetson/Desktop/data/cyq/slam/guide_nav2_ws/src/guide_nav2/config/controller_server.yaml"
    # planner_server_yaml = "/home/jetson/Desktop/data/cyq/slam/guide_nav2_ws/src/guide_nav2/config/planner_server.yaml"
    # recoveries_server_yaml = "/home/jetson/Desktop/data/cyq/slam/guide_nav2_ws/src/guide_nav2/config/recoveries_server.yaml"
    map_file = "/home/jetson/Desktop/data/cyq/slam/guide_nav2_ws/src/guide_nav2/maps/map-0619.yaml"
    pbstream_path = '/home/jetson/Desktop/data/cyq/slam/guide_nav2_ws/src/guide_nav2/maps/map-0619.pbstream'
    # urdf_name = "wheelchair_base.urdf"
    # wheelchair_description_pkg_share = FindPackageShare('wheelchair_description').find('wheelchair_description') 
    # urdf_model_path = os.path.join(wheelchair_description_pkg_share, f'urdf/{urdf_name}')

    # Wheelchair description launch file
    wheelchair_launch_file = os.path.join(get_package_share_directory('wheelchair_description'), 'launch', 'wheelchair_description.launch.py')
    oradar_launch_file = os.path.join(get_package_share_directory('oradar_lidar'), 'launch', 'ms200_scan_view.launch.py')
    imu_launch_file = os.path.join(get_package_share_directory('wit_ros2_imu'), 'launch', 'rviz_and_imu.launch.py')

    configuration_directory = '/home/jetson/Desktop/data/cyq/slam/guide_nav2_ws/src/guide_nav2/config'
    configuration_basename = 'guide_nav2.lua'



    return LaunchDescription(
        [
            # ------ 坐标变换 节点 ------ #
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='base_footprint_to_base_link',
                arguments=['0','0','-0.54','0','0','0','base_link','base_footprint']
            ),

            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='base_link_to_base_imu',
                arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link']
            ),    
            
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='base_link_to_base_laser',
                arguments=['0','0','0.18','0','0','0','base_link','lidar']
            ),

            # ------ cartographer 传感器 节点 ------ #
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(wheelchair_launch_file)
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(oradar_launch_file)
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(imu_launch_file)
            ),

            Node(
                package = 'cartographer_ros',
                executable = 'cartographer_node',
                arguments = [
                    '-configuration_directory', configuration_directory,
                    '-load_state_filename', pbstream_path,
                    '-configuration_basename', configuration_basename],
                remappings = [
                    ('scan', 'scan'),
                    ('imu', 'imu/data_raw')
                    ],
                parameters=[{'use_sim_time': False}],
                output = 'screen'
            ),

            Node(
                package = 'cartographer_ros',
                executable = 'occupancy_grid_node',
                parameters = [
                    {'use_sim_time': False},
                    {'resolution': 0.05}],
            ),


            # ------ nav2 导航节点 ------ #
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[{'use_sim_time': False}, 
                            {'yaml_filename':map_file}]
            ),

            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                parameters=[amcl_yaml]
            ),
            
            Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                output='screen',
                parameters=[controller_server_yaml]
            ),

            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                parameters=[planner_server_yaml]
            ),
                
            Node(
                package='nav2_recoveries',
                executable='recoveries_server',
                name='recoveries_server',
                parameters=[recoveries_server_yaml],
                output='screen'
            ),

            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                parameters=[bt_navigator_yaml]
            ),

            Node(
                package='rviz2',
                # namespace='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen'
            ),

            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                parameters=[{'use_sim_time': False},
                            {'autostart': True},
                            {'node_names': ['map_server', 
                                            'amcl',
                                            'planner_server',
                                            'controller_server',
                                            'recoveries_server'                                        ,
                                            'bt_navigator'
                                            ]}]
            )
        ]
    )

