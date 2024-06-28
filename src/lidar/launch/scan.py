#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  # LiDAR publisher node
  lidar_scan = Node(
      package='lidar',
      executable='scan',
      name='lidar',
      output='screen',
      parameters=[
        {'device_model': 'MS200'},
        {'frame_id': 'lidar'},
        {'scan_topic': '/scan'},
        {'port_name': '/dev/oradar'},
        {'baudrate': 230400},
        {'angle_min': 0.0},
        {'angle_max': 360.0},
        {'range_min': 0.05},
        {'range_max': 20.0},
        {'clockwise': False},
        {'motor_speed': 10}
      ]
  )

  # # base_link to laser_frame tf node
  lidar_tf = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='lidar_stf',
    arguments=['0','0','0.18','0','0','0','base_link','lidar']
  )

  # Define LaunchDescription variable
  ord = LaunchDescription([lidar_tf, lidar_scan])
  
  return ord
