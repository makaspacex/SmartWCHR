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
-- slam2d.lua 是为实现cartographer建图所创建的，使用了手动编写发布的里程计信息
-- 参考了/opt/ros/eloquent/share/cartographer_ros/configuration_files/backpack_2d.lua

--------------------------------------------------------------------------------
-- slam_2d.lua : no odom
-- slam2d.lua : with odom
--------------------------------------------------------------------------------
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",                        -- 构建地图所使用的坐标系
  tracking_frame = "base_footprint",              -- *SLAM算法跟踪的帧的ROS帧ID，如果要使用IMU，它应该在这个地方被选用，尽管它可能会漂移(imu_link)
  published_frame = "odom",            -- *要用作发布坐标的子帧的ROS帧ID --lidar
  odom_frame = "odom",         -- 仅当provide_odom_frame为true时使用，通常是“odom”。
  provide_odom_frame = false,
  publish_frame_projected_to_2d = false,    -- 如果enable，则发布姿态将严格限制在纯2D位姿下
  use_pose_extrapolator = true,
  use_odometry = true,                      -- 如果enable，则订阅topic为odom中的nav_msgs/Odometry，这种情况下必须提供【里程计】
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
}

MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.min_range = 0.1                   -- 设置激光雷达数据的最小有效范围
TRAJECTORY_BUILDER_2D.max_range = 8.                   -- 设置激光雷达数据的最大有效范围
TRAJECTORY_BUILDER_2D.use_imu_data = false              -- 设置是否使用 IMU 数据

------------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------  调整子图构建参数 --------------------------------------------------------- ---
------------------------------------------------------------------------------------------------------------------------------------------
-- 控制生成每个子图所需的激光扫描帧数。增加此数值可以提高子图的稳定性和准确性，但会增加内存消耗和计算复杂度。
-- 默认值35
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 60

-- TRAJECTORY_BUILDER_2D.num_accumulated_range_data 参数是用于控制在进行激光扫描匹配前累积的激光扫描数据（即范围数据）的数量。
-- 这个参数对于决定如何处理和利用传入的激光扫描数据至关重要。
-- 控制生成每个子图所需的激光扫描帧数。增加此数值可以提高子图的稳定性和准确性，但会增加内存消耗和计算复杂度。
-- 主要作用是指定在进行一次扫描匹配或创建一个新的子图前，需要累积多少帧激光扫描数据。这种累积机制允许 Cartographer 在进行地图构建或位置更新之前，
-- 整合多个时间点的数据，从而提高地图的准确性和鲁棒性。具体来说：
-- 1) 数据整合：通过累积多帧数据，可以获得更加完整和详细的环境信息。这对于在动态环境中维持稳定的定位尤为重要。
-- 2) 噪声抑制：累积多帧数据有助于平滑和减少单个扫描中的噪声影响，因为多个数据点的平均效果可以减少偶发误差的影响。
-- 3) 计算需求：累积更多的数据可能会增加处理每次扫描所需的计算资源，因此需要在数据质量和处理速度之间找到平衡。

-- 典型值可能从 1 到数十不等，具体取决于：
-- 1) 传感器的频率：传感器数据的频率高，可能需要累积较少的数据就能获得足够的环境信息。
-- 2) 机器人的速度：快速移动的机器人可能需要更频疑的数据更新，因此可能设置较低的累积值。
-- 3) 计算资源：如果计算资源有限，可能需要限制累积的数据量以保持实时性。

-- 默认值和调整建议
-- 2D 为10
-- 在默认配置中，num_accumulated_range_data 的值通常由 Cartographer 的配置文件或在启动时设置的参数决定。
-- 如果没有特别的需求，可以使用默认值开始。如果在实际应用中发现定位精度不足或处理延迟较大，可以考虑调整这个参数值。
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 20

------------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------  调整扫描匹配器参数 -----------------------------------------------------------
------------------------------------------------------------------------------------------------------------------------------------------
-- 定义在进行实时相关扫描匹配时，线性和角度搜索的范围。这些参数对于调整扫描匹配的灵活性和鲁棒性至关重要。
-- linear_search_window通常设置在 0.1 米到 0.3 米之间。默认值，2D: 0.1, 3D: 0.15
-- angular_search_window: 通常设置在 10 度到 30 度之间。2D默认值math.rad(20.),3D默认值math.rad(1.)
-- TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
-- TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.)

-- 用来控制在实时相关扫描匹配过程中，平移误差对总成本函数的影响权重。这个参数帮助调整平移变化对匹配过程的重要性。
-- translation_delta_cost_weight 的主要作用是衡量在进行扫描匹配时，平移误差对匹配成本函数的贡献。
-- 具体来说，这个权重决定了在计算匹配成本时，平移误差的影响程度。该参数的值越高，算法越倾向于减少平移误差，即优先考虑减少平移差异。
-- 高值: 如果 translation_delta_cost_weight 设置得较高，那么匹配算法会更加重视减少平移误差。这在平移误差对系统性能影响较大的应用场景中特别有用，如在机器人需要精确定位的环境中。
-- 低值: 较低的权重值会使平移误差在成本计算中的影响减小，这可能有助于在存在一定平移噪声的情况下提供更稳定的角度匹配。
-- 默认值，2D: 1e-1, 3D: 1e-1
-- TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1e-1

------------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------  里程计权重 ------------------------------------------------------------------ 
------------------------------------------------------------------------------------------------------------------------------------------
-- 调整此参数来控制里程计数据在后端优化中的权重。降低此值可以减少里程计误差对最终SLAM结果的影响。
-- 用于设置在位姿图优化问题中里程计数据的权重。这个权重决定了里程计数据在总体SLAM解决方案中的影响力，关系到定位的精度和地图的质量。
-- Cartographer 的配置可能因版本和使用的具体配置（如2D或3D）而异。通常，对于2D SLAM，里程计权重的默认值可能是： 1e5 或 1e5
-- 在3D SLAM配置中，这个值可能会有所不同，通常为了适应三维空间中更复杂的动态和错误模型，权重可能会设置得更高或更低。
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e6
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e6
-- fastlio, pose
-- pointlio, pose
-- livox mid360, ladar-imu.
-- real sense, 


------------------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------  实时回环检测 ----------------------------------------------------------------
------------------------------------------------------------------------------------------------------------------------------------------
-- 是否使用实时回环检测
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

-- 此参数决定了何种程度的回环检测被认为是有效的。增加此值可以减少错误的回环检测，从而减少累积误差。
POSE_GRAPH.constraint_builder.min_score = 0.65

-- 调整此值以改善全局定位的准确性。调整这个参数时，需要考虑到回环检测的敏感性和误报的可能性。值越低，接受的回环检测越多，可能包括一些错误的检测；值越高，则只有高置信度的匹配才会被接受。
-- 通常，这个参数的默认值可能在 0.55 到 0.6 范围内，这是基于经验和一些典型场景的常用设置。
-- POSE_GRAPH.constraint_builder.global_localization_min_score= 0.6

return options
