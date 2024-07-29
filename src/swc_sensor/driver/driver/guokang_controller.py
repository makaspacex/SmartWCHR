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

from datetime import datetime

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
        # self.ser_write_timer_ = self.create_timer(0.1, self.timerCallback)

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
        def get_sign(hex_str):
            if len(hex_str.split(" ")) != 11:
                raise Exception("error data")
            sum = 0
            for _ele in hex_str.split(" "):
                sum += int(_ele,16)
            return str(hex(sum))[-2:]

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


