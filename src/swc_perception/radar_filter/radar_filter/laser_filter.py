import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import array
import math



class LaserFilter(Node):
    def __init__(self):
        super().__init__('laser_filter')
        
        # 单独声明每个参数
        self.declare_parameter('start_angle', 180)
        self.declare_parameter('end_angle', 70)
        self.declare_parameter('sub_topic', "scan_s2_raw")
        self.declare_parameter('pub_topic', "scan_s2_filter")
        
        sub_topic = self.get_parameter('sub_topic').get_parameter_value().string_value
        pub_topic = self.get_parameter('pub_topic').get_parameter_value().string_value
        
        self.subscription = self.create_subscription(
            LaserScan,
            sub_topic,
            self.listener_callback,
            10)
        
        self.publisher = self.create_publisher(
            LaserScan,
            pub_topic,
            10)

    def listener_callback(self, msg:LaserScan):
        # 从参数服务器获取角度参数
        start_angle_degrees = self.get_parameter('start_angle').get_parameter_value().integer_value
        end_angle_degrees = self.get_parameter('end_angle').get_parameter_value().integer_value
        
        # 将角度从度转换为弧度
        start_angle_rad = start_angle_degrees / 180.0 * math.pi
        end_angle_rad = end_angle_degrees / 180.0 * math.pi

        # 计算要保留的角度范围的索引
        start_index = int((start_angle_rad - msg.angle_min) / msg.angle_increment)
        end_index = int((end_angle_rad - msg.angle_min) / msg.angle_increment)
        
        ranges = np.array(msg.ranges)

        if end_index > start_index:
            ranges[:start_index] = math.inf
            ranges[end_index:] = math.inf
        else:
            # 270~90   270~360  0~90 保留    90~270 去掉  
            ranges[end_index:start_index+1] = math.inf   
            # ranges[start_index:] = math.inf
            # ranges[:end_index] = math.inf

        # 仅保留指定角度范围内的数据
        msg.ranges = array.array('f', ranges.tolist())
        
        if msg.intensities:
            intensities = np.array(msg.intensities)
            intensities[:start_index] = -math.inf
            intensities[end_index:] = -math.inf
            msg.intensities = array.array('f', intensities.tolist())
       
        # 发布过滤后的数据
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    laser_filter = LaserFilter()
    rclpy.spin(laser_filter)
    laser_filter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()