import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction, TimerAction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, PythonExpression
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals, IfCondition, UnlessCondition
from pathlib import Path
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    package_name = Path(__file__).parent.parent.stem
    package_share_dir = get_package_share_directory(package_name)
    
    # Get the launch directory
    pb_swc_simulation_launch_dir = os.path.join(get_package_share_directory('pb_swc_simulation'), 'launch')
    navigation2_launch_dir = os.path.join(get_package_share_directory('swc_navigation'), 'launch')

    # Create the launch configuration variables
    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_lio_rviz = LaunchConfiguration('lio_rviz')
    use_nav_rviz = LaunchConfiguration('nav_rviz')

    ################################ robot_description parameters start ###############################
    launch_params = yaml.safe_load(open(os.path.join(
    get_package_share_directory(package_name), 'config', 'reality', 'measurement_params_real_gk01.yaml')))
    robot_description = Command(['xacro ', os.path.join(
    get_package_share_directory(package_name), 'urdf', 'gkchair01_base.urdf'),
    ' xyz:=', launch_params['base_link2livox_frame']['xyz'], ' rpy:=', launch_params['base_link2livox_frame']['rpy']])
    ################################# robot_description parameters end ################################

    ########################## linefit_ground_segementation parameters start ##########################
    segmentation_params = os.path.join(package_share_dir, 'config', 'reality', 'segmentation_real.yaml')
    ########################## linefit_ground_segementation parameters end ############################

    #################################### FAST_LIO parameters start ####################################
    fastlio_mid360_params = os.path.join(package_share_dir, 'config', 'reality', 'fastlio_mid360_real.yaml')
    fastlio_mid360_params_dict = yaml.safe_load(open(fastlio_mid360_params))
    fastlio_rviz_cfg_dir = os.path.join(package_share_dir, 'rviz', 'fastlio.rviz')
    fastlio_pub_tf_en = fastlio_mid360_params_dict['/**']['ros__parameters']['publish']['tf_en']
    lio_remappings = [("/Odometry", "/fastlio_odomtry")]  if fastlio_pub_tf_en else None
    ##################################### FAST_LIO parameters end #####################################
    
    ################################### POINT_LIO parameters start ####################################
    pointlio_mid360_params = os.path.join(package_share_dir, 'config', 'reality', 'pointlio_mid360_real.yaml')
    pointlio_rviz_cfg_dir = os.path.join(package_share_dir, 'rviz', 'pointlio.rviz')
    
    #################################### POINT_LIO parameters end #####################################
    
    ################################## slam_toolbox parameters start ##################################
    slam_toolbox_map_file = PathJoinSubstitution([package_share_dir, 'map', world])
    slam_toolbox_param_file = os.path.join(package_share_dir, 'config', 'reality', 'slam_toolbox.yaml')
    ################################### slam_toolbox parameters end ###################################

    ################################### navigation2 parameters start ##################################
    nav2_map_name = PathJoinSubstitution([package_share_dir, 'map',world]), ".yaml"
    nav2_params_file = os.path.join(package_share_dir, 'config', 'reality', 'nav2_params_real_gk01.yaml')
    ################################### navigation2 parameters end ####################################

    ################################ icp_registration parameters start ################################
    icp_pcd_file = PathJoinSubstitution([package_share_dir, 'PCD', world]), ".pcd"
    icp_registration_params_file = os.path.join(package_share_dir, 'config', 'simulation', 'icp_registration_sim.yaml')
    ################################# icp_registration parameters end #################################

    ############################# pointcloud_downsampling parameters start ############################
    pointcloud_downsampling_config_file = os.path.join(package_share_dir, 'config', 'reality', 'pointcloud_downsampling_real.yaml')
    ############################# pointcloud_downsampling parameters start ############################

    ####################### Livox_ros_driver2 parameters start #######################
    xfer_format   = 4    # 0-PointCloud2Msg(PointXYZRTL), 1-LivoxCustomMsg, 2-PclPxyziMsg, 3-LivoxImuMsg, 4-AllMsg
    multi_topic   = 0    # 0-All LiDARs share the same topic, 1-One LiDAR one topic
    data_src      = 0    # 0-lidar, others-Invalid data src
    publish_freq  = 10.0 # freqency of publish, 5.0, 10.0, 20.0, 50.0, etc.
    output_type   = 0
    frame_id      = 'livox_frame'
    lvx_file_path = '/home/livox/livox_test.lvx'
    cmdline_bd_code = 'livox0000000001'

    cur_path = os.path.split(os.path.realpath(__file__))[0] + '/'
    cur_config_path = cur_path + '../config'
    user_config_path = os.path.join(cur_config_path, 'reality', 'MID360_config.json')

    livox_ros2_params = [
        {"xfer_format": xfer_format},
        {"multi_topic": multi_topic},
        {"data_src": data_src},
        {"publish_freq": publish_freq},
        {"output_data_type": output_type},
        {"frame_id": frame_id},
        {"lvx_file_path": lvx_file_path},
        {"user_config_path": user_config_path},
        {"cmdline_input_bd_code": cmdline_bd_code}
    ]
    ####################### Livox_ros_driver2 parameters end #########################

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use reality (Gazebo) clock if true')

    declare_use_lio_rviz_cmd = DeclareLaunchArgument(
        'lio_rviz',
        default_value='False',
        description='Visualize FAST_LIO or Point_LIO cloud_map if true')
    
    declare_nav_rviz_cmd = DeclareLaunchArgument(
        'nav_rviz',
        default_value='True',
        description='Visualize navigation2 if true')

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='328',
        description='Select world (map file, pcd file, world file share the same name prefix as the this parameter)')

    declare_mode_cmd = DeclareLaunchArgument(
        'mode',
        default_value='',
        description='Choose mode: nav, mapping')
    
    declare_LIO_cmd = DeclareLaunchArgument(
        'lio',
        default_value='fast_lio',
        description='Choose lio alogrithm: fastlio or pointlio')

    declare_localization_cmd = DeclareLaunchArgument(
        'localization',
        default_value='',
        description='Choose localization method: slam_toolbox, amcl, icp')

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
        }],
        output='screen'
    )

    # Specify the actions
    start_livox_ros_driver2_node = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params
    )

    bringup_imu_complementary_filter_node = Node(
        package='imu_complementary_filter',
        executable='complementary_filter_node',
        name='complementary_filter_gain_node',
        output='screen',
        parameters=[
            {'do_bias_estimation': True},
            {'do_adaptive_gain': True},
            {'use_mag': False},
            {'gain_acc': 0.01},
            {'gain_mag': 0.01},
        ],
        remappings=[
            ('/imu/data_raw', '/livox/imu'),
        ]
    )

    bringup_linefit_ground_segmentation_node = Node(
        package='linefit_ground_segmentation_ros',
        executable='ground_segmentation_node',
        output='screen',
        parameters=[segmentation_params]
    )

    bringup_pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
        remappings=[('cloud_in',  ['/segmentation/obstacle']),
                    ('scan',  ['/scan'])],
        parameters=[{
            'target_frame': 'livox_frame',
            'transform_tolerance': 0.01,
            'min_height': -1.0,
            'max_height': 0.1,
            'angle_min': -3.14159,  # -M_PI/2
            'angle_max': 3.14159,   # M_PI/2
            'angle_increment': 0.0043,  # M_PI/360.0
            'scan_time': 0.3333,
            'range_min': 0.45,
            'range_max': 10.0,
            'use_inf': True,
            'inf_epsilon': 1.0
        }],
        name='pointcloud_to_laserscan'
    )
    
    bringup_pcl_filter_node = Node(
        package='pcl_filter', 
        executable='pcl_filter_node',
        name="pcl_filter_node"
    )

    bringup_LIO_group = GroupAction([
      
        GroupAction(
            condition = LaunchConfigurationEquals('lio', 'fastlio'),
            actions=[
            Node(
                package='fast_lio',
                executable='fastlio_mapping',
                name='fastlio_mapping',
                remappings=lio_remappings,
                parameters=[
                    fastlio_mid360_params,
                    {use_sim_time: use_sim_time}
                ],
                output='screen'
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', fastlio_rviz_cfg_dir,'--ros-args', '--log-level', 'WARN'],
                condition = IfCondition(use_lio_rviz),
            ),
        ]),

        GroupAction(
            condition = LaunchConfigurationEquals('lio', 'pointlio'),
            actions=[
            Node(
                package='point_lio',
                executable='pointlio_mapping',
                name='pointlio_mapping',
                output='screen',
                parameters=[
                    pointlio_mid360_params,
                    {'use_sim_time': use_sim_time,
                    'use_imu_as_input': False,  # Change to True to use IMU as input of Point-LIO
                    'prop_at_freq_of_imu': True,
                    'check_satu': True,
                    'init_map_size': 10,
                    'point_filter_num': 3,  # Options: 1, 3
                    'space_down_sample': True,
                    'filter_size_surf': 0.5,  # Options: 0.5, 0.3, 0.2, 0.15, 0.1
                    'filter_size_map': 0.5,  # Options: 0.5, 0.3, 0.15, 0.1
                    'ivox_nearby_type': 6,   # Options: 0, 6, 18, 26
                    'runtime_pos_log_enable': False}
                ],
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', pointlio_rviz_cfg_dir,'--ros-args', '--log-level', 'WARN'],
                condition = IfCondition(use_lio_rviz),
            )
        ])
    ])

    start_localization_group = GroupAction(
        condition = LaunchConfigurationEquals('mode', 'nav'),
        actions=[
            Node(
                condition = LaunchConfigurationEquals('localization', 'slam_toolbox'),
                package='slam_toolbox',
                executable='localization_slam_toolbox_node',
                name='localization_slam_toolbox_node',
                parameters=[
                    slam_toolbox_param_file,
                    {'use_sim_time': use_sim_time,
                    'map_file_name': slam_toolbox_map_file,
                    'map_start_pose': [0.0, 0.0, 0.0]}
                ],
            ),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(navigation2_launch_dir,'localization_amcl_launch.py')),
                condition = LaunchConfigurationEquals('localization', 'amcl'),
                launch_arguments = {
                    'use_sim_time': use_sim_time,
                    'params_file': nav2_params_file,
                    'map': nav2_map_name}.items()
            ),

            TimerAction(
                period=7.0,
                actions=[
                    Node(
                        condition=LaunchConfigurationEquals('localization', 'icp'),
                        package='icp_registration',
                        executable='icp_registration_node',
                        output='screen',
                        parameters=[
                            icp_registration_params_file,
                            {'use_sim_time': use_sim_time,
                                'pcd_path': icp_pcd_file}
                        ],
                        # arguments=['--ros-args', '--log-level', ['icp_registration:=', 'DEBUG']]
                    )
                ]
            ),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(navigation2_launch_dir, 'map_server_launch.py')),
                condition = LaunchConfigurationNotEquals('localization', 'slam_toolbox'),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'map': nav2_map_name,
                    'params_file': nav2_params_file,
                    'container_name': 'nav2_container'}.items())
        ]
    )

    bringup_fake_vel_transform_node = Node(
        package='fake_vel_transform',
        executable='fake_vel_transform_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'spin_speed': 0.0 # rad/s
        }]
    )
    
    bringup_robot_localization_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        remappings=[("/odometry/filtered", "/odom")],
        parameters=[os.path.join(package_share_dir, 'config/reality/ekf_fast_lio.yaml')],
        condition = IfCondition(PythonExpression(['not ',f"{fastlio_pub_tf_en}"]))
    )

    start_mapping = Node(
        condition = LaunchConfigurationEquals('mode', 'mapping'),
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='async_slam_toolbox_node',
        parameters=[
            slam_toolbox_param_file,
            {'use_sim_time': use_sim_time,}
        ],
    )

    start_navigation2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(navigation2_launch_dir, 'bringup_swc_navigation.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': nav2_map_name,
            'params_file': nav2_params_file,
            'nav_rviz': use_nav_rviz}.items()
    )
    
    # fastlio里程计转换
    fastlio_convert_node = Node(
        package='fastlio_odomcovert',
        executable='covert_node',
        name="fastlio_odomcovert",
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[("/covert_odomtry", "/odom")],
        output='screen'
    )

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_lio_rviz_cmd)
    ld.add_action(declare_nav_rviz_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_mode_cmd)
    ld.add_action(declare_localization_cmd)
    ld.add_action(declare_LIO_cmd)
    
    
    # 放到sensor里面了
    # ld.add_action(start_robot_state_publisher_cmd)
    # ld.add_action(start_livox_ros_driver2_node)
    # ld.add_action(bringup_imu_complementary_filter_node)
    # ld.add_action(bringup_linefit_ground_segmentation_node)
    # ld.add_action(bringup_pcl_filter_node)
    # ld.add_action(bringup_pointcloud_to_laserscan_node)
    # ld.add_action(bringup_fake_vel_transform_node)
    
    ld.add_action(bringup_LIO_group)
    ld.add_action(fastlio_convert_node) # 处理fastlio的定位tf变换以及里程消息
    ld.add_action(bringup_robot_localization_node) # 使用lio原生的tf定位的话，就可以关掉
    ld.add_action(start_localization_group)
    ld.add_action(start_mapping)
    ld.add_action(start_navigation2)

    return ld
