import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, LaunchConfigurationEquals, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument
from pathlib import Path
from launch.substitutions import  Command


def generate_launch_description():
    package_name = Path(__file__).parent.parent.stem
    package_share_dir = get_package_share_directory(package_name)

    robot_name = LaunchConfiguration("robot_name")
    launch_ros_local = LaunchConfiguration("launch_ros_local")
    launch_encoder = LaunchConfiguration("launch_encoder")
    use_sim_time = LaunchConfiguration('use_sim_time',default="false")
    ####################### Livox_ros_driver2 parameters start #######################
    xfer_format = 4  # 0-PointCloud2Msg(PointXYZRTL), 1-LivoxCustomMsg, 2-PclPxyziMsg, 3-LivoxImuMsg, 4-AllMsg
    multi_topic = 0  # 0-All LiDARs share the same topic, 1-One LiDAR one topic
    data_src = 0  # 0-lidar, others-Invalid data src
    publish_freq = 10.0  # freqency of publish, 5.0, 10.0, 20.0, 50.0, etc.
    output_type = 0
    frame_id = "livox_frame"
    lvx_file_path = "/home/livox/livox_test.lvx"
    cmdline_bd_code = "livox0000000001"

    cur_path = os.path.split(os.path.realpath(__file__))[0] + "/"
    cur_config_path = cur_path + "../config"
    user_config_path = os.path.join(cur_config_path, "reality", "MID360_config.json")

    livox_ros2_params = [
        {"use_sim_time": use_sim_time},
        {"xfer_format": xfer_format},
        {"multi_topic": multi_topic},
        {"data_src": data_src},
        {"publish_freq": publish_freq},
        {"output_data_type": output_type},
        {"frame_id": frame_id},
        {"lvx_file_path": lvx_file_path},
        {"user_config_path": user_config_path},
        {"cmdline_input_bd_code": cmdline_bd_code},
    ]
    ####################### Livox_ros_driver2 parameters end #########################
    
    # Specify the actions
    start_livox_ros_driver2_node = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params,
        condition=UnlessCondition(use_sim_time),
    )

    bringup_imu_complementary_filter_node = Node(
        package='imu_complementary_filter',
        executable='complementary_filter_node',
        name='complementary_filter_gain_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'publish_tf': False},
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
    segmentation_params = os.path.join(package_share_dir, 'config', 'reality', 'segmentation_real.yaml')
    bringup_linefit_ground_segmentation_node = Node(
        package='linefit_ground_segmentation_ros',
        executable='ground_segmentation_node',
        name="ground_segmentation",
        output='screen',
        parameters=[segmentation_params,{"use_sim_time":use_sim_time}]
    )
    
    bringup_pcl_filter_node = Node(
        package='pcl_filter', 
        executable='pcl_filter_node',
        name="pcl_filter_node",
        parameters=[{"use_sim_time":use_sim_time,"lidar_topic":"/livox/lidar/pointcloud","pointcloud_output_topic":"/pcl_filter"}]
    )
    
    bringup_pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
        remappings=[('cloud_in',  ['/segmentation/obstacle']),
                    ('scan',  ['/scan'])],
        parameters=[{
            "use_sim_time":use_sim_time,
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
    
    bringup_fake_vel_transform_node = Node(
        package='fake_vel_transform',
        executable='fake_vel_transform_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'spin_speed': 0.0 # rad/s
        }]
    )
    
    ################################ robot_description parameters start ###############################
    launch_params = yaml.safe_load(open(os.path.join(package_share_dir, 'config', 'reality', 'measurement_params_real_gk01.yaml')))
    robot_description = Command(['xacro ', os.path.join(package_share_dir, 'urdf', 'gkchair01_base.urdf'),' xyz:=', launch_params['base_link2livox_frame']['xyz'], ' rpy:=', launch_params['base_link2livox_frame']['rpy']])
    ################################# robot_description parameters end ################################
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            "use_gui": False,
            'robot_description': robot_description
        }],
        output='screen',
        condition=UnlessCondition(use_sim_time),
    )
    start_joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            "use_gui": False,
            'robot_description': robot_description
        }],
        output='screen',
        condition=UnlessCondition(use_sim_time),
    )
    
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
                default_value="false",
                description="launch_driver",
            ),
            DeclareLaunchArgument(
                "launch_ros_local",
                default_value="false",
                description="start robot localization",
            ),
            DeclareLaunchArgument(
                "launch_encoder",
                default_value="false",
                description="start encoder sensor",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="if use_sim_time",
            ),
            # 启用手柄控制
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("joy_ctrl"), "launch", "joy_ctrl.py"
                    ),
                ),
                launch_arguments={'use_sim_time': use_sim_time,"robot_name": robot_name}.items(),
                condition=UnlessCondition(use_sim_time),
            ),
            # 启动编码器
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("encoder"), "launch", "odom.py"
                    ),
                ),
                launch_arguments={'use_sim_time': use_sim_time,"robot_name": robot_name}.items(),
                condition=IfCondition(launch_encoder),
            ),
            # 启动驱动器
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("driver"), "launch", "driver.py"
                    ),
                ),
                launch_arguments={'use_sim_time': use_sim_time,"robot_name": robot_name}.items(),
                condition=UnlessCondition(use_sim_time),
            ),
            # 启动相机
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("gen_camera"), "launch", "camera.py"
                    )
                ),
                launch_arguments={'use_sim_time': use_sim_time}.items(),
                condition=UnlessCondition(use_sim_time),
            ),
            # 启动ms200雷达
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("lidar_ms200"), "launch", "scan.py"
                    )
                ),
                launch_arguments={'use_sim_time': use_sim_time}.items(),
                condition=UnlessCondition(use_sim_time),
            ),
            Node(
                package="radar_filter",
                executable="radar_filter",
                name="laser_filter_ms200",
                parameters=[{
                    'use_sim_time': use_sim_time,
                    "start_angle": 110,
                    "end_angle": 250,
                    }],
                output="screen",
                 remappings=[
                    ("/scan_raw", "/scan_ms200_raw"),
                    ("/scan_filter", "/scan_ms200_filter"),
                ],
                condition=IfCondition(use_sim_time),
            ),
            # 启动思岚s2雷达
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("sllidar"), "launch", "scan.py"
                    )
                ),
                condition=UnlessCondition(use_sim_time),
            ),
            Node(
                package="radar_filter",
                executable="radar_filter",
                name="laser_filter_s2",
                parameters=[{
                    'use_sim_time': use_sim_time,
                    "start_angle": 190,
                    "end_angle": 80,
                    }],
                output="screen",
                 remappings=[
                    ("/scan_raw", "/scan_s2_raw"),
                    ("/scan_filter", "/scan_s2_filter"),
                ],
                condition=IfCondition(use_sim_time),
            ),
            # # 启动imu
            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource(
            #         os.path.join(get_package_share_directory("imu"), "launch", "imu.py")
            #     ),
            #     condition=UnlessCondition(use_sim_time),
            # ),
            
            # 启动robot_state_publisher
            start_joint_state_publisher_cmd,
            start_robot_state_publisher_cmd,
            
            # 启动mid360
            start_livox_ros_driver2_node,
            
            # imu过滤
            bringup_imu_complementary_filter_node,
            
            # pcl过滤
            bringup_pcl_filter_node,
            
            # 点云分割
            bringup_linefit_ground_segmentation_node,
            
            # 点云转scan
            bringup_pointcloud_to_laserscan_node,
            
            # 虚拟的base_link
            # bringup_fake_vel_transform_node
        ]
    )
