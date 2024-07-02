

from re import L
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import serial

class ControllerNode(Node):
    """控制节点"""
    def __init__(self, portname="/dev/ttyUSB0", rate=115200):
        super().__init__('driver')
        
        self.lineSpeed = 0.0                    # 初始化线速度
        self.angleSpeed = 0.0                   # 初始化角速度
        self.intervalWheel = 0.570              # 左右轮间距（m）
        self.FrameHead_Macro = 0x42             # 帧头
        self.SpeedControll_Macro = 0x01         # 速度控制地址
        self.MaxSpeed = 0.5                    # 最大速度
        self.buffer = 12 * [0]                  # 串口发送内容

        try:
            # 创建接口对象 串口设备名：'/dev/ttyUSB0' 比特率：115200
            self.ser = serial.Serial(portname, rate, timeout=1000)
        except Exception as e:
            print("串口异常：", e)

        # 接收cmd数据、计算发送数据
        self.cmd_sub_ = self.create_subscription(Twist, 'cmd_vel', self.CMDCallBack, 1)
        # 发送buffer
        self.ser_write_timer_ = self.create_timer(0.05, self.SpeedCallBack)
        self.get_logger().info('Hi from controller')

    def BuffToSerial(self, sendData, len):
        check = 0x0

        check += self.FrameHead_Macro
        check += self.SpeedControll_Macro
        check += len
        for data in sendData:
            check += data

        self.buffer[0] = self.FrameHead_Macro
        self.buffer[1] = self.SpeedControll_Macro
        self.buffer[2] = len
        self.buffer[3:3+8] = sendData
        self.buffer[len-1] = check & 0b11111111
 
    def CMDCallBack(self, msg: Twist):
        sendData = 8 * [0]

        self.lineSpeed = msg.linear.x
        self.angleSpeed = msg.angular.z 

        # v = (Vr + Vl) / 2 线速度公式
        # w = (Vr - Vl) / l 角速度公式
        leftSpeedData = self.lineSpeed - 0.5 * self.angleSpeed * self.intervalWheel
        rightSpeedData = self.lineSpeed + 0.5 * self.angleSpeed * self.intervalWheel
        # if leftSpeedData > self.MaxSpeed or rightSpeedData > self.MaxSpeed:
        #     # 相当于归一化 
        #     max_lr_speed = max(leftSpeedData, rightSpeedData)
        #     leftSpeedData  = leftSpeedData  / max_lr_speed
        #     rightSpeedData = rightSpeedData / max_lr_speed
        leftSpeedData = int((leftSpeedData / self.MaxSpeed) * 1500).to_bytes(4, byteorder='big', signed=True)
        rightSpeedData = int((rightSpeedData / self.MaxSpeed) * 1500).to_bytes(4, byteorder='big', signed=True)

        sendData[0:4] = leftSpeedData
        sendData[4:]  = rightSpeedData
        self.get_logger().info(f"\033[32mmsg:{msg} leftSpeedData:{leftSpeedData} rightSpeedData:{rightSpeedData}\033[0m")
        
        # self.BuffToSerial(sendData, 12)
        # self.ser.write(self.buffe)

    def SpeedCallBack(self):
        self.ser.write(self.buffer)

def main(args = None):
    rclpy.init(args=args)
    controller = ControllerNode()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

