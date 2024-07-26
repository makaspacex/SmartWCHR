
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from message_filters import ApproximateTimeSynchronizer, Subscriber
import math
import numpy as np
import array
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
import sys
from urdf_parser_py.urdf import URDF


def radar_filter(msg, start_angle_degrees, end_angle_degrees):
    start_angle_rad = start_angle_degrees / 180.0 * math.pi
    end_angle_rad = end_angle_degrees / 180.0 * math.pi

    start_index = int((start_angle_rad - msg.angle_min) / msg.angle_increment)
    end_index = int((end_angle_rad - msg.angle_min) / msg.angle_increment)
    
    ranges = np.array(msg.ranges)

    if start_index < end_index:
        ranges[:start_index] = math.inf
        ranges[end_index:] = math.inf
    else:
        ranges[end_index:start_index] = math.inf

    msg.ranges = array.array('f', ranges.tolist())
    
    if msg.intensities:
        intensities = np.array(msg.intensities)
        if start_index < end_index:
            intensities[:start_index] = -math.inf
            intensities[end_index:] = -math.inf
        else:
            intensities[end_index:start_index] = -math.inf
        msg.intensities = array.array('f', intensities.tolist())
    
    return msg

def interpolate_and_align(lidar1_msg, lidar2_msg):
    # 找到最小的 angle_increment 和对应的角度范围
    min_angle_increment = min(lidar1_msg.angle_increment, lidar2_msg.angle_increment)
    
    # 计算新的角度范围和点数
    total_angle_range = max(lidar1_msg.angle_max, lidar2_msg.angle_max) - min(lidar1_msg.angle_min, lidar2_msg.angle_min)
    num_points = int(total_angle_range / min_angle_increment) + 1
    
    new_angles = np.linspace(min(lidar1_msg.angle_min, lidar2_msg.angle_min), max(lidar1_msg.angle_max, lidar2_msg.angle_max), num_points)
    
    # 插值 lidar1_msg.ranges 和 lidar2_msg.ranges
    lidar1_ranges = np.array(lidar1_msg.ranges)
    lidar2_ranges = np.array(lidar2_msg.ranges)
    lidar1_angles = np.linspace(lidar1_msg.angle_min, lidar1_msg.angle_max, len(lidar1_ranges))
    lidar2_angles = np.linspace(lidar2_msg.angle_min, lidar2_msg.angle_max, len(lidar2_ranges))
    
    lidar1_interpolated_ranges = np.interp(new_angles, lidar1_angles, lidar1_ranges, left=math.inf, right=math.inf)
    lidar2_interpolated_ranges = np.interp(new_angles, lidar2_angles, lidar2_ranges, left=math.inf, right=math.inf)
    
    # 插值 lidar1_msg.intensities 和 lidar2_msg.intensities
    if lidar1_msg.intensities and lidar2_msg.intensities:
        lidar1_intensities = np.array(lidar1_msg.intensities)
        lidar2_intensities = np.array(lidar2_msg.intensities)
        
        lidar1_interpolated_intensities = np.interp(new_angles, lidar1_angles, lidar1_intensities, left=-math.inf, right=-math.inf)
        lidar2_interpolated_intensities = np.interp(new_angles, lidar2_angles, lidar2_intensities, left=-math.inf, right=-math.inf)
        
        merged_intensities = np.maximum(lidar1_interpolated_intensities, lidar2_interpolated_intensities)
    else:
        merged_intensities = []
    
    # 合并 ranges
    merged_ranges = np.minimum(lidar1_interpolated_ranges, lidar2_interpolated_ranges)
    
    return merged_ranges, merged_intensities, min_angle_increment, min(lidar1_msg.angle_min, lidar2_msg.angle_min), max(lidar1_msg.angle_max, lidar2_msg.angle_max)

class TFListener(Node):
    def __init__(self):
        super().__init__('tf_listener')
        self.tf_buffer = Buffer()
        self.listener = TransformListener(self.tf_buffer, self)

    def get_transform(self, target_frame, source_frame):
        try:
            # 等待变换关系变得可用
            self.tf_buffer.can_transform(target_frame, source_frame, rclpy.time.Time())
            # 获取变换关系
            transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
            return transform
        except Exception as e:
            self.get_logger().info(f'Could not transform {target_frame} to {source_frame}: {str(e)}')
            return None

class DualLidarSyncNode(Node):
    def __init__(self):
        super().__init__('dual_lidar_sync_node')
        
        self.lidar_sub1 = Subscriber(self, LaserScan, '/scan_s2_rawdd')
        self.lidar_sub2 = Subscriber(self, LaserScan, '/scan_ms200_raw')

        self.declare_parameter('lidar_1_start_angle', 180)
        self.declare_parameter('lidar_1_end_angle', 70)
        self.declare_parameter('lidar_2_start_angle', 0)
        self.declare_parameter('lidar_2_end_angle', 210)
        
        self.ts = ApproximateTimeSynchronizer(
            [self.lidar_sub1, self.lidar_sub2], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.lidar_callback)
        
        self.lidar_pub = self.create_publisher(LaserScan, 'merged_lidar', 10)

        self.s2_filter_lidar_pub = self.create_publisher(LaserScan, 'scan_s2_filter', 10)
        self.ms200_filter_lidar_pub = self.create_publisher(LaserScan, 'scan_ms200_filter', 10)


        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # tf_listener = TFListener()
        # self.laser_transform = tf_listener.get_transform('base_link', 'camera_link')
        # Set a timer to periodically check the transform
        # self.timer = self.create_timer(1.0, self.timer_callback)
        
    def lidar_callback(self, lidar1_msg, lidar2_msg):
        # now = rclpy.time.Time()
        # now = self.get_clock().now().to_msg()
        # transform = self.tf_buffer.lookup_transform('laser_link', 'laser_ms200_link', now)
        # self.get_logger().info(f'Translation: {transform.transform.translation}')
        # self.get_logger().info(f'Rotation: {transform.transform.rotation}')


        lidar_1_start_angle = self.get_parameter('lidar_1_start_angle').get_parameter_value().integer_value
        lidar_1_end_angle = self.get_parameter('lidar_1_end_angle').get_parameter_value().integer_value
        lidar_2_start_angle = self.get_parameter('lidar_2_start_angle').get_parameter_value().integer_value
        lidar_2_end_angle = self.get_parameter('lidar_2_end_angle').get_parameter_value().integer_value

        lidar1_msg = radar_filter(lidar1_msg, lidar_1_start_angle, lidar_1_end_angle)
        lidar2_msg = radar_filter(lidar2_msg, lidar_2_start_angle, lidar_2_end_angle)


        self.s2_filter_lidar_pub.publish(lidar1_msg)
        self.ms200_filter_lidar_pub.publish(lidar2_msg)

        merged_ranges, merged_intensities, angle_increment, angle_min, angle_max = interpolate_and_align(lidar1_msg, lidar2_msg)

        merged_msg = LaserScan()
        merged_msg.header.stamp = self.get_clock().now().to_msg()
        merged_msg.header.frame_id = lidar1_msg.header.frame_id

        merged_msg.angle_min = angle_min
        merged_msg.angle_max = angle_max
        merged_msg.angle_increment = angle_increment
        merged_msg.time_increment = lidar1_msg.time_increment
        merged_msg.scan_time = lidar1_msg.scan_time
        merged_msg.range_min = min(lidar1_msg.range_min, lidar2_msg.range_min)
        merged_msg.range_max = max(lidar1_msg.range_max, lidar2_msg.range_max)

        merged_msg.ranges = array.array('f', merged_ranges.tolist())
        if merged_intensities.any():
            merged_msg.intensities = array.array('f', merged_intensities.tolist())

        self.lidar_pub.publish(merged_msg)
        self.get_logger().info('Published merged lidar message')

def main(args=None):
    rclpy.init(args=args)
    node = DualLidarSyncNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
