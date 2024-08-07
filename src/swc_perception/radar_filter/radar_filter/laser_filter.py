import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import array
import math
from builtin_interfaces.msg import Time

from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from rclpy.qos import qos_profile_sensor_data


class LaserFilter(Node):
    def __init__(self):
        super().__init__('laser_filter')
        
        # 单独声明每个参数
        self.declare_parameter('start_angle', 185)
        self.declare_parameter('end_angle', 70)
        
        self.subscription = self.create_subscription(
            LaserScan,
            "scan_raw",
            self.listener_callback,
            qos_profile_sensor_data)
        
        self.publisher = self.create_publisher(
            LaserScan,
            "scan_filter",
            1)

        # 从参数服务器获取角度参数
        self.start_angle_degrees = self.get_parameter('start_angle').get_parameter_value().integer_value
        self.end_angle_degrees = self.get_parameter('end_angle').get_parameter_value().integer_value

    def listener_callback(self, msg:LaserScan):
        # msg.header.stamp = self.get_clock().now().to_msg()
        filtered_ranges = None
        
        # 将角度从度转换为弧度
        start_angle_rad = self.start_angle_degrees / 180.0 * math.pi
        end_angle_rad = self.end_angle_degrees / 180.0 * math.pi

        # 计算要保留的角度范围的索引
        start_index = int((start_angle_rad - msg.angle_min) / msg.angle_increment)
        end_index = int((end_angle_rad - msg.angle_min) / msg.angle_increment)
        
        ranges = np.array(msg.ranges)
        
        if end_index > start_index:
            ranges[:start_index] = math.inf
            ranges[end_index:] = math.inf
        else:
            ranges[end_index:start_index+1] = math.inf   

        # 仅保留指定角度范围内的数据
        filtered_ranges = array.array('f', ranges.tolist())
        
        filtered_intensities = None
        if msg.intensities:
            intensities = np.array(msg.intensities)
            intensities[:start_index] = -math.inf
            intensities[end_index:] = -math.inf
            filtered_intensities = array.array('f', intensities.tolist())
        
        # 创建新的LaserScan消息对象
        filtered_msg = LaserScan()
        filtered_msg.header = msg.header
        filtered_msg.angle_min = msg.angle_min
        filtered_msg.angle_max = msg.angle_max
        filtered_msg.angle_increment = msg.angle_increment
        filtered_msg.time_increment = msg.time_increment
        filtered_msg.scan_time = msg.scan_time
        filtered_msg.range_min = msg.range_min
        filtered_msg.range_max = msg.range_max
        filtered_msg.ranges = filtered_ranges
        filtered_msg.intensities = filtered_intensities

        # 发布过滤后的数据
        self.publisher.publish(filtered_msg)
        

def main(args=None):
    rclpy.init(args=args)
    laser_filter = LaserFilter()
    rclpy.spin(laser_filter)
    laser_filter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()