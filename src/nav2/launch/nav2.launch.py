"""
This module provides navigation function for wheelchair.

Functions:
- generate_launch_description(): launch nodes for navigation.
"""

from pathlib import Path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """
    Function to load nav2 server config files and launch nav2 nodes.
    """
    # load servers config file

    nav2_package_path = Path(FindPackageShare('nav2').find('nav2'))
    guide_yaml = str( nav2_package_path / Path("config/guide.yaml"))

    # load servers config file separately
    # amcl_yaml = str( nav2_package_path / Path("config/amcl.yaml"))
    # bt_navigator_yaml = str( nav2_package_path / Path("config/bt_navigator.yaml"))
    # controller_server_yaml = str( nav2_package_path / Path("config/controller_server.yaml"))
    # planner_server_yaml = str( nav2_package_path / Path("config/planner_server.yaml"))
    # behavior_server_yaml = str( nav2_package_path / Path("config/behavior_server.yaml"))

    # amcl_yaml = os.path.join(get_package_share_directory('nav2'), 'config', 'amcl.yaml')
    # bt_navigator_yaml = os.path.join(get_package_share_directory('nav2'), 'config', 'bt_navigator.yaml')
    # controller_server_yaml = os.path.join(get_package_share_directory('nav2'), 'config', 'controller_server.yaml')
    # planner_server_yaml = os.path.join(get_package_share_directory('nav2'), 'config', 'planner_server.yaml')
    # recoveries_server_yaml = os.path.join(get_package_share_directory('nav2'), 'config', 'recoveries_server.yaml')

    # acml_yaml = "/home/jetson/Desktop/data/cyq/slam/nav2_ws/src/nav2/config/acml.yaml"
    # bt_navigator_yaml = "/home/jetson/Desktop/data/cyq/slam/nav2_ws/src/nav2/config/bt_navigator.yaml"
    # controller_server_yaml = "/home/jetson/Desktop/data/cyq/slam/nav2_ws/src/nav2/config/controller_server.yaml"
    # planner_server_yaml = "/home/jetson/Desktop/data/cyq/slam/nav2_ws/src/nav2/config/planner_server.yaml"
    # recoveries_server_yaml = "/home/jetson/Desktop/data/cyq/slam/nav2_ws/src/nav2/config/recoveries_server.yaml"

    # # map_file = "/home/jetson/Desktop/data/cyq/slam/nav2_ws/src/nav2/maps/map-0619.yaml"
    # map_file = os.path.join(get_package_share_directory('nav2'), 'maps', 'map-0619.yaml')
    # # pbstream_path = '/home/jetson/Desktop/data/cyq/slam/nav2_ws/src/nav2/maps/map-0619.pbstream'
    # pbstream_path = os.path.join(get_package_share_directory('nav2'), 'maps', 'map-0619.pbstream')

    # load slam_2d map file
    # ! ! ! TODO : map file path need to be optimized to slam_2d map path ! ! !
    map_file = str( nav2_package_path / Path("maps/map-0619.yaml"))
    pbstream_path = str( nav2_package_path / Path("maps/map-0619.pbstream"))

    # urdf_name = "wheelchair_base.urdf"
    # wcmodel_pkg_share = FindPackageShare('wcmodel').find('wcmodel') 
    # urdf_model_path = os.path.join(wcmodel_pkg_share, f'urdf/{urdf_name}')


    # load nav2 config lua file
    configuration_directory = str( nav2_package_path / Path("config"))
    configuration_basename = 'nav2.lua'
    # load wheelchair description launch file
    wcmodel_package_path = Path(FindPackageShare('wcmodel').find('wcmodel'))
    wheelchair_launch_file = str( wcmodel_package_path / Path("launch/wc_base.py"))

    rviz_config_path = str( Path(nav2_package_path ) / Path("config/guide.rviz"))
    # # load wheelchair description launch file
    # wheelchair_launch_file = os.path.join(get_package_share_directory('wcmodel'), 'launch', 'wc_base.py')
    # oradar_launch_file = os.path.join(get_package_share_directory('lidar'), 'launch', 'scan_view.py')
    # imu_launch_file = os.path.join(get_package_share_directory('imu'), 'launch', 'start.py')

    # configuration_directory = os.path.join(get_package_share_directory('nav2'), 'config')
    # # configuration_directory = '/home/jetson/Desktop/data/cyq/slam/nav2_ws/src/nav2/config'
    # configuration_basename = 'nav2.lua'


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
            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource(oradar_launch_file)
            # ),
            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource(imu_launch_file)
            # ),

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
                executable = 'cartographer_occupancy_grid_node',
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
                # parameters=[amcl_yaml]
                parameters=[guide_yaml]
            ),

            Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                output='screen',
                # parameters=[controller_server_yaml]
                parameters=[guide_yaml]
            ),

            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                # parameters=[planner_server_yaml]
                parameters=[guide_yaml]
            ),

            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                # parameters=[behavior_server_yaml],
                parameters=[guide_yaml]
            ),

            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                # parameters=[bt_navigator_yaml]
                parameters=[guide_yaml]
            ),

            Node(
                package='rviz2',
                # namespace='rviz2',
                executable='rviz2',
                arguments=['-d', rviz_config_path],
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
                                            'behavior_server',
                                            'bt_navigator'
                                            ]}]
            )
        ]
    )
