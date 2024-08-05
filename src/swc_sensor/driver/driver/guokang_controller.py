from re import L
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from apscheduler.schedulers.background import BackgroundScheduler

import serial
import math
import time
from statistics import mean
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from rclpy.qos import qos_profile_sensor_data
import re
from nav_msgs.msg import Odometry
import tf_transformations
from geometry_msgs.msg import Quaternion, TransformStamped

from datetime import datetime
from dataclasses import dataclass

@dataclass
class SWCStatus:
    linear_x:float         # 线速度          
    angle_z:float          # 角速度  
    pose_x:float           # 坐标x 
    pose_y:float           # 坐标y 
    ryw_z:float            # 偏航角z  
    battery_remain:float   # 电池剩余
    def __init__(self):
        super().__init__()

def get_sign(hex_str:str):
    sum = 0
    hex_str = hex_str.replace(" ","")
    for _ele in re.findall(r'[A-F\d]{2}', hex_str):
        sum += int(_ele,16)
    return str(hex(sum)).upper()[-2:]

def verify_data(frame_str:str):
    ss = get_sign(frame_str[:-2])
    if ss == frame_str[-2:]:
        return True
    return False

def decode_msg(valid_msg:str) -> SWCStatus:
    ret = SWCStatus()
    eles = re.findall(r'[A-F\d]{2}', valid_msg)
    
    ret.linear_x = int.from_bytes(bytes.fromhex("".join(eles[2:4])),byteorder='big', signed=True)/1000          # 线速度
    ret.angle_z = int.from_bytes(bytes.fromhex("".join(eles[4:6])),byteorder='big', signed=True)/1.0            # 角速度
    ret.pose_x = int.from_bytes(bytes.fromhex("".join(eles[6:10])),byteorder='big', signed=True)/1000           # 坐标x
    ret.pose_y = int.from_bytes(bytes.fromhex("".join(eles[10:14])),byteorder='big', signed=True)/1000          # 坐标y
    ret.ryw_z = int.from_bytes(bytes.fromhex("".join(eles[14:16])),byteorder='big', signed=False)/1.0           # 偏航角z
    ret.battery_remain = int.from_bytes(bytes.fromhex("".join(eles[16:18])),byteorder='big', signed=True)/100   # 电池剩余
    return ret


class CmdVelToSerial(Node):
    def __init__(self, portname="/dev/driver", baudrate = 9600):
        super().__init__('cmd_vel_to_serial')
        
        self.portname = portname
        self.baudrate = baudrate
        
        # 串口配置
        self.ser = None
        self.connect_dev()
        

        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            qos_profile_sensor_data
        )
        # self.subscription  # prevent unused variable warning
        # self.twist_pub_callback_timer = self.create_timer(0.1, self.twist_pub_callback)
        self.all_buffer = ""
        self.odom_publisher = self.create_publisher(Odometry, '/odom_gk', qos_profile_sensor_data)
        
        self.last_write_time = math.inf
        self.zero_data = "AF 01 00 00 00 00 01 00 00 00 00"

        # 记录刚刚接收到的几条速度指令
        self.linear_queue = [1.0] * 4
        self.angular_queue = [1.0] * 4
        self.index_queue = 0

        self.get_logger().info(f'gk01 controller is here!')
        
        # 用来控制打印输出频率的
        self.last_print_time = time.time()
        
        self.log_f = open('/home/jetson/Desktop/SmartWCHR/runtime/cmd_log.txt', 'w+')
        
    def twist_pub_callback(self):
        self.connect_dev()
        try:
            _raw_data = self.ser.read_all()
            self.all_buffer += _raw_data.hex().upper()
        except Exception as e:
            print(e)
        
        msgs = re.findall(r"(BF01[A-F\d]{52})",self.all_buffer)[::-1]
        # 无消息，不发布
        if len(msgs)== 0:
            return

        valid_msg = None
        for msg in msgs:
            if verify_data(msg):
                valid_msg = msg
                break
        # 没有有效消息，不发布
        if not valid_msg:
            return
        
        swc_state = decode_msg(valid_msg)
        
        # 发布里程计数据
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'
        odom_msg.pose.pose.position.x = swc_state.pose_x
        odom_msg.pose.pose.position.y = swc_state.pose_y
        odom_msg.twist.twist.linear.x = swc_state.linear_x
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = swc_state.angle_z
        
        # 将欧拉角转换为四元数（roll, pitch, yaw）
        quaternion = tf_transformations.quaternion_from_euler(0.0, 0.0, swc_state.ryw_z)
        quat_msg = Quaternion()
        quat_msg.x = quaternion[0]
        quat_msg.y = quaternion[1]
        quat_msg.z = quaternion[2]
        quat_msg.w = quaternion[3]
        odom_msg.pose.pose.orientation = quat_msg
        
        self.odom_publisher.publish(odom_msg)
        
        # 超过500条就保留最后10条
        if len(self.all_buffer) > 56*500:
            self.all_buffer = self.all_buffer[-560:]
        
    def connect_dev(self):
        if hasattr(self,"ser") and self.ser is not None:
            return 
        while True:
            try:
                self.ser = serial.Serial(self.portname, self.baudrate, timeout=1)
                break
            except Exception:
                self.get_logger().error(f"connect to {self.portname} is failed, tring again....")
                time.sleep(1)
    
    def get_ser_data(self, v_linear, v_angular):

        base_data = self.zero_data        
        # 乘以1000并转换为16进制
        v_linear_hex = int(v_linear * 1000).to_bytes(2, byteorder='big', signed=True).hex().upper()
        v_angular_hex = int(v_angular * 1000).to_bytes(2, byteorder='big', signed=True).hex().upper()

        # 将base_data转换为列表以便替换字符
        base_data_list = base_data.split()

        # 替换线速度的字符
        base_data_list[2] = v_linear_hex[:2]
        base_data_list[3] = v_linear_hex[2:]

        # 替换角速度的字符
        base_data_list[4] = v_angular_hex[:2]
        base_data_list[5] = v_angular_hex[2:]

        data_str = ' '.join(base_data_list)

        hex_string = f"{data_str} {get_sign(data_str)}".replace(" ","")
        ser_data = bytes.fromhex(hex_string)

        return ser_data
        
    def write_to_ser(self, ser_data):
        try:
            self.ser.write(ser_data)
        except Exception:
            self.ser = None
            self.get_logger().error(f"failed write data to {self.portname}")

    def listener_callback(self, msg):
        # 保留1s的平均速率，如果等于0，忽略后面的0速度指令
        self.connect_dev()
        
        linear_speed = msg.linear.x
        angular_speed = msg.angular.z
       

        self.linear_queue[self.index_queue] = linear_speed
        self.angular_queue[self.index_queue] = angular_speed
        self.index_queue = (self.index_queue + 1) % len(self.angular_queue)

        
        if math.isclose(mean(self.linear_queue), 0.0) and math.isclose(mean(self.angular_queue), 0.0):
            return
        
        # 写入一些内容到文件中
        # time = time.time()
        # 获取当前时间的时间戳
        timestamp = time.time()
        # 将时间戳转换为datetime对象
        datetime_obj = datetime.fromtimestamp(timestamp)
        self.log_f.write(f'{datetime_obj}\t{msg.linear}\t{msg.angular}\n')
        self.log_f.flush()
    
        
        # 将接收到的速度转化为串口数据
        ser_data = self.get_ser_data(linear_speed, angular_speed)
        self.write_to_ser(ser_data=ser_data)
        
        now_time = time.time()
        if now_time - self.last_print_time > 1:
            self.get_logger().debug(f'linear_speed:{linear_speed}   angular_speed:{angular_speed}')
            self.last_print_time = now_time

def main(args=None):
    rclpy.init(args=args)

    cmd_vel_to_serial = CmdVelToSerial()

    rclpy.spin(cmd_vel_to_serial)

    # 清理串口
    cmd_vel_to_serial.ser.close()

    cmd_vel_to_serial.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


