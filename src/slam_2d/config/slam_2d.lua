-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

-- use '--' instead of '//' to add notes in .lua files
-- slam_2d.lua 是为实现cartographer建图所创建的
-- 参考了/opt/ros/eloquent/share/cartographer_ros/configuration_files/backpack_2d.lua

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",                        -- 构建地图所使用的坐标系
  tracking_frame = "imu_link",              -- *SLAM算法跟踪的帧的ROS帧ID，如果要使用IMU，它应该在这个地方被选用，尽管它可能会漂移(imu_link)
  published_frame = "base_link",                -- *要用作发布坐标的子帧的ROS帧ID --lidar
  odom_frame = "odom",                      -- 仅当provide_odom_frame为true时使用，通常是“odom”。
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,    -- 如果enable，则发布姿态将严格限制在纯2D位姿下
  use_odometry = false,                     -- 如果enable，则订阅topic为odom中的nav_msgs/Odometry，这种情况下必须提供【里程计】
  use_nav_sat = false,                      -- 如果enable，则订阅主题为fix中的sensor_msgs/NavSatFix，这种情况下必须要使用【导航数据】
  use_landmarks = false,                    -- 如果enable，则订阅主题为landmarks中的cartographer_ros_msgs/LandmarkList，必须提供LandmarkLists数据
  num_laser_scans = 1,                      -- *要订阅的laser scan的主题数量
  num_multi_echo_laser_scans = 0,           -- *要订阅的multi-echo laser scan的主题数量
  num_subdivisions_per_laser_scan = 1,      -- *将每个接收到的（多回波）激光扫描分成的点云数
  num_point_clouds = 0,                     -- 要订阅的point cloud的主题数量
  lookup_transform_timeout_sec = 0.2,       -- 用于使用tf2查找转换的超时秒数
  submap_publish_period_sec = 0.3,          -- 发布子图姿势的时间间隔（以秒为单位）
  pose_publish_period_sec = 5e-3,           -- 发布姿势的时间间隔（以秒为单位），例如 5e-3 表示频率为 200 Hz。
  trajectory_publish_period_sec = 30e-3,    -- 发布轨迹标记的时间间隔（以秒为单位），例如 30e-3 30 毫秒。
  rangefinder_sampling_ratio = 1.,          -- 测距仪消息的固定比率采样
  odometry_sampling_ratio = 1.,             -- 里程计消息的固定比率采样
  fixed_frame_pose_sampling_ratio = 1.,     -- 固定帧消息的固定比率采样
  imu_sampling_ratio = 1.,                  -- IMU消息的固定比率采样
  landmarks_sampling_ratio = 1.,            -- 地标消息的固定比率采样
  -- pure_localization = true,                 -- 设置纯定位模式
}

MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 10   -- 设置每次更新子地图时积累的激光雷达扫描数量 10
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 35       -- 设置每个子地图中包含的激光雷达扫描次数 35
TRAJECTORY_BUILDER_2D.min_range = 0.3                   -- 设置激光雷达数据的最小有效范围
TRAJECTORY_BUILDER_2D.max_range = 12.                   -- 设置激光雷达数据的最大有效范围
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.      -- 设置缺失数据射线的长度
TRAJECTORY_BUILDER_2D.use_imu_data = true              -- 设置是否使用 IMU 数据
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true   -- 设置是否使用实时回环检测

-- --尽量小点；如果移动距离或旋转过小, 或者时间过短, 不进行地图的更新
-- TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.05
-- TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)
-- TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.2




-- TRAJECTORY_BUILDER.pure_localization = true


TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1 -- 设置在线相关扫描匹配的线性搜索窗口大小
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.    -- 设置平移增量的代价权重
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1      -- 设置旋转增量的代价权重

POSE_GRAPH.optimization_problem.huber_scale = 1e2   -- 设置 Huber 损失函数的尺度参数
POSE_GRAPH.optimize_every_n_nodes = 10              -- 设置每隔多少个节点进行一次图优化 35
POSE_GRAPH.constraint_builder.min_score = 0.65      -- 设置生成约束时的最小匹配分数



return options
