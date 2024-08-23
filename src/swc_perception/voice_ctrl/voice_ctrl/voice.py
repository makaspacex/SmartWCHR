import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.timer import Timer
import serial
import json
import time

woshuo = b'\xa5\x01\x01\x04\x00\x00\x00\xa5\x00\x00\x00\xb0'
queren = b'\xA5\x01\xff\x04\x00\x00\x00\xA5\x00\x00\x00\xB2'

class CommandSubscriber(Node):

    def __init__(self):
        super().__init__('command_subscriber')
        self.state = None  # 状态变量，初始为None
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # 定时器，100ms 回调一次
        self.timer = self.create_timer(0.1, self.timer_callback)

        try:
            # 打开串口
            self.ser = serial.Serial("/dev/xfserial", 115200, 8, 'N', 1, timeout=5)
            print("打开串口")
        except Exception as e:
            print("---打开串口异常---：",e)

        self.buf_flag = 0
        self.buf = []
        self.first = 0

        self.current_timer = None  # 保存当前的定时器对象


    def listener_callback(self, command):
        if command == "xiao3 fei1 qian2 jin4":
            self.state = "go"
            print("前进")
            self.current_timer = self.create_timer(3.0, self.reset_state)
        elif command == "xiao3 fei1 zuo3 zhuan3":
            self.state = "left"
            print("左转")
            self.current_timer = self.create_timer(3.14, self.reset_state)
        elif command == "xiao3 fei1 you4 zhuan3":
            self.state = "right"
            print("右转")
            self.current_timer = self.create_timer(3.14, self.reset_state)


    def timer_callback(self):
        # 读取串口数据
        rcv = self.ser.read_all()
        len_r = len(rcv)

        if rcv == woshuo:
            self.ser.write(queren)
            print("#####")
        elif(len_r > 1) :
            self.buf.append(rcv)
            self.buf_flag = 1

        elif len_r < 1 and self.buf_flag==1:
            self.buf_flag = 0
            data_list = self.buf
            self.buf = []
            self.process_serial_data(data_list)

        # 发布速度指令
        twist = Twist()
        if self.state == "go":
            twist.linear.x = 0.3  # 前进速度
        elif self.state == "left":
            twist.angular.z = 0.25  # 左转速度
        elif self.state == "right":
            twist.angular.z = -0.25  # 右转速度
        else:
            return
        self.publisher_.publish(twist)

    def process_serial_data(self, data_list):
        str1 = str(data_list)

        f_data = str1.find('{') #从左侧开始找
        l_data = str1.rfind('}') #从右侧开始找
        str1 = str1[f_data:l_data+1]

        str1 = str1.replace("\\","")
        str1 = str1.replace("', b'","")
        str1 = str1.replace('"{',"{")
        str1 = str1.replace('}"',"}")


        try:
            json_str = json.loads(str1)
        except json.JSONDecodeError as e:
            print(f"JSON decoding failed: {e}")
            return
        
        print("-**-"*20)
        
        if 'code' in json_str and self.first == 0:
            sss = json_str['content']
            print(json_str['content'])   
            self.first = 1
        else:
            command = json_str['content']['result']
            self.listener_callback(command)
            print("你说的话是："+ command) #获取语音唤醒的结果


    def cancel_current_timer(self):
        if self.current_timer is not None:
            self.current_timer.cancel()
            self.current_timer = None

    def reset_state(self):
        self.state = None
        self.cancel_current_timer()
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        for i in range(3):
            self.publisher_.publish(twist)


def main(args=None):
    rclpy.init(args=args)

    command_subscriber = CommandSubscriber()

    rclpy.spin(command_subscriber)

    command_subscriber.ser.close()
    command_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
