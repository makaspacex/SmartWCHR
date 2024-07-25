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
        {'frame_id': 'laser_link'},
        {'scan_topic': '/scan_ms200_raw'},
        {'port_name': '/dev/oradar'},
        {'baudrate': 230400},
        {'angle_min': 0.0},
        {'angle_max': 360.0},
        {'range_min': 0.1},
        {'range_max': 10.0},
        {'clockwise': False},
        {'motor_speed': 10}
      ]
  )
  
  # Define LaunchDescription variable
  ord = LaunchDescription([lidar_scan])
  
  return ord
