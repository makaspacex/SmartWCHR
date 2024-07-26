"""
This module provides navigation function for wheelchair.

Functions:
- generate_launch_description(): launch nodes for navigation.
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """
    Function to load nav2 server config files and launch nav2 nodes.
    """

    nav2_package_path = Path(FindPackageShare('nav2').find('nav2'))
    guide_yaml = str( nav2_package_path / Path("config/guide.yaml"))
    guide_yaml = "/home/jetson/Desktop/SmartWCHR/src/nav2/config/guide.yaml"
    remappings = [
            ('scan', 'scan'),
            ('imu', 'imu/data_raw')]
    rviz_config_path = str( Path(nav2_package_path ) / Path("config/nav2_default_view.rviz"))
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
        )

    swc_nav_bringup_package_path = Path(FindPackageShare('swc_nav_bringup').find('swc_nav_bringup'))
    wheelchair_launch_file = str( swc_nav_bringup_package_path / Path("launch/wc_base.py"))
    swc_nav_bringup_node = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(wheelchair_launch_file)
            )
    
    map_file = str( nav2_package_path / Path("maps/map_fx.yaml"))
    # map_file = str( nav2_package_path / Path("maps/map-0619.yaml"))
    # map_file = str( nav2_package_path / Path("maps/map_jidianlou-01.yaml"))
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': False},
                    {'yaml_filename':map_file}]
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        remappings=remappings,
        # parameters=[amcl_yaml]
        parameters=[guide_yaml]
    )
    
    
    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        remappings=remappings,
        parameters=[guide_yaml]
    )

    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        remappings=remappings,
        parameters=[guide_yaml]
    )

    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        remappings=remappings,
        parameters=[guide_yaml]
    )

    behavior_path = str( Path(nav2_package_path ) / Path("config/behavior.xml"))
    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        remappings=remappings,
        parameters=[guide_yaml,
                    {'default_nav_to_pose_bt_xml':behavior_path}
                    ]
    )

    lifecycle_manager_server = Node(
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

    rviz_first_node = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=rviz_node,
            on_exit=[   swc_nav_bringup_node,
                        map_server_node,
                        amcl_node,
                        planner_server_node,
                        controller_server_node,
                        behavior_server_node,
                        bt_navigator_node,
                        lifecycle_manager_server
                        ]
        )
    )
    return LaunchDescription(
        [
            # rviz_node,
            # rviz_first_node,
            swc_nav_bringup_node,
            map_server_node,
            amcl_node,
            planner_server_node,
            controller_server_node,
            behavior_server_node,
            bt_navigator_node,
            lifecycle_manager_server
        ]
    )
