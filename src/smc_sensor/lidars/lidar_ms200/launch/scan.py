#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  package_name = Path(__file__).parent.parent.stem
  package_share_dir = get_package_share_directory(package_name)
  
  # LiDAR publisher node
  lidar_scan = Node(
      package=package_name,
      executable='scan',
      name='lidar',
      output='screen',
      parameters=[
        {'device_model': 'MS200'},
        {'frame_id': 'laser_ms200_link'},
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
