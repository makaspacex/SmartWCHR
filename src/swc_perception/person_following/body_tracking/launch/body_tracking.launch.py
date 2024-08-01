# Copyright (c) 2022，Horizon Robotics.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('hand_gesture_detection'),
                    'launch/hand_gesture_detection.launch.py'))
        ),
        Node(
            package='body_tracking',
            executable='body_tracking',
            output='screen',
            parameters=[
                {"activate_wakeup_gesture": 1},
                {"img_width": 640},
                {"img_height": 480},
                {"track_serial_lost_num_thr": 30},
                {"move_step": 0.5},
                {"rotate_step": 0.5},
                {"activate_robot_move_thr": 5}
            ],
            arguments=['--ros-args', '--log-level', 'info']
        )
    ])
