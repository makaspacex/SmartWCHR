from re import L
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import serial
import math
import time

class SpeedController:
    def __init__(self, max_acceleration=0.05):
        self.current_speed = 0.0  # 当前速度
        self.max_acceleration = max_acceleration  # 最大加速度限制
        self.last_update_time = time.time()  # 上次更新时间
        self.time_to_hold = 0.1  # 保持当前速度的时间，秒
        self.last_command_time = time.time()  # 上次接收指令的时间

    def update_speed(self, target_speed=None):
        # 更新时间
        current_time = time.time()
        time_interval = current_time - self.last_update_time
        self.last_update_time = current_time

        # 检查是否收到新的速度指令
        if target_speed is not None:
            self.last_command_time = current_time
            self.target_speed = target_speed
        else:
            # 如果超过保持时间没有新指令，则目标速度设置为0
            if (current_time - self.last_command_time) > self.time_to_hold:
                self.target_speed = 0

        # 计算最大速度变化
        max_speed_change = self.max_acceleration * time_interval

        # 计算加速度
        desired_change = self.target_speed - self.current_speed
        if abs(desired_change) > max_speed_change:
            desired_change = max_speed_change if desired_change > 0 else -max_speed_change

        # 更新速度
        self.current_speed += desired_change
        return self.current_speed




class ControllerNode(Node):
    """控制节点"""
    def __init__(self, portname="/dev/driver", rate=115200):
        super().__init__('driver')
        
        self.lineSpeed = 0.0                    # 初始化线速度
        self.angleSpeed = 0.0                   # 初始化角速度
        self.intervalWheel = 0.570              # 左右轮间距（m）
        self.FrameHead_Macro = 0x42             # 帧头
        self.SpeedControll_Macro = 0x01         # 速度控制地址
        self.MaxSpeed = 0.5                    # 最大速度
        self.MIN_SPEED = 0.022                  # 最小速度
        self.buffer = 12 * [0]                  # 串口发送内容
        
        self.ser = None
        try:
            # 创建接口对象 串口设备名：'/dev/driver' 比特率：115200
            self.ser = serial.Serial(portname, rate, timeout=1000)
        except Exception as e:
            print("串口异常：", e)
            raise e

        # 接收cmd数据、计算发送数据
        self.cmd_sub_ = self.create_subscription(Twist, 'cmd_vel', self.CMDCallBack, 1)
        # 每隔0.05s向串口发送buffer                    修改成0.01s，看是否能够更加灵敏一些
        self.ser_write_timer_ = self.create_timer(0.1, self.SpeedCallBack)
        self.get_logger().info('Hi from controller')

        # 当前状态   stop, starting, running, stopping四种
        self.state = 'stop'
        # 当前发送到串口的左右轮速度
        self.left_speed = 0
        self.right_speed = 0


        self.left_Control = SpeedController() 
        self.right_Control = SpeedController()


        self.last_print_time = time.time()


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
        # sendData = 8 * [0]

        self.lineSpeed = msg.linear.x
        self.angleSpeed = msg.angular.z 

        # v = (Vr + Vl) / 2 线速度公式
        # w = (Vr - Vl) / l 角速度公式
        leftSpeedData = self.lineSpeed - 0.5 * self.angleSpeed * self.intervalWheel
        rightSpeedData = self.lineSpeed + 0.5 * self.angleSpeed * self.intervalWheel
        leftSpeedData *= 1.12

        self.left_speed = leftSpeedData
        self.right_speed = rightSpeedData

        # self.left_speed = self.left_Control.update_speed(leftSpeedData)
        # self.right_speed = self.right_Control.update_speed(rightSpeedData)


    def SpeedCallBack(self):
        if not self.ser:
            return
        
        sendData = 8 * [0]

        leftSpeedData = int((self.left_speed / self.MaxSpeed) * 1500).to_bytes(4, byteorder='big', signed=True)
        rightSpeedData = int((self.right_speed / self.MaxSpeed) * 1500).to_bytes(4, byteorder='big', signed=True)

        sendData[0:4] = leftSpeedData
        sendData[4:]  = rightSpeedData


        curtime = time.time()
        if curtime - self.last_print_time > 1:
            self.get_logger().info("left_speed:" + str(self.left_speed) + '         right_speed:' + str(self.right_speed))
            self.last_print_time = curtime
        
        self.BuffToSerial(sendData, 12)
        
        self.ser.write(self.buffer)

def main(args = None):
    rclpy.init(args=args)
    controller = ControllerNode()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

