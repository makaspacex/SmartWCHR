# 要通过传感器数据计算得到里程计（Odometry）的数据，我们通常需要从编码器、IMU（惯性测量单元）、GPS或其他传感器获取输入。
# 在这个例子中，我将展示如何使用轮式机器人的轮子编码器计算其位置和速度，然后发布这些数据作为里程计信息。
# 我们将使用Python在ROS 2框架中实现这一过程。前提条件: 
# 假设你有一个简单的两轮驱动机器人，每个轮子上有一个编码器。每个编码器可以测量轮子转动的距离或步数。

# 需要的数据
# - 轮子的半径 R
# - 两轮之间的距离 L
# - 每个编码器的计数 (每个时间周期的增量)

# 编码器到里程计的计算步骤
# - 计算每个轮子的线速度：如果你知道编码器的分辨率（每圈的脉冲数）和轮子的半径，你可以计算每个轮子的线速度。
# - 计算机器人的线速度和角速度：使用两个轮子的速度，你可以计算整个机器人的线速度和角速度。
# - 更新机器人的位置和姿态：利用线速度和角速度，通过简单的运动学可以更新机器人的位置和姿态。


# 关于编码器
# 在轮式机器人的应用中，轮子上常用的编码器一般是旋转编码器（Rotary Encoder），有时也叫作轴编码器。
# 这种设备用来测量轮子转动的角度或者旋转的步数，从而可以推算出轮子转动的距离。旋转编码器主要分为两种类型：
# 1、增量式编码器（Incremental Encoder）
# 增量式编码器提供相对位置信息，意味着它们输出关于轴旋转的增量变化（通常是脉冲信号）。机器人或任何系统需要从已知的参考点开始计数，才能确定当前位置。
# 它们是通过生成脉冲来工作的，每旋转一定角度生成一个脉冲信号。通过计数这些脉冲，可以推算出轮子的转动距离。
# 
# 2、绝对式编码器（Absolute Encoder）
# 绝对式编码器提供每个可能位置的唯一代码，无论设备是否有电源，它都能提供精确的位置信息。
# 即使在断电或重新启动后，绝对式编码器也能提供精确的位置信息，不需要重新校准或找到参考点。
# 在轮式机器人中，增量式编码器因其成本较低和简单性而更常见。编码器通过测量轮子上的旋转来计算位置和速度，而不是直接测量角度（尽管从技术上讲，它们确实测量了旋转的角度）。编码器的输出通常是数字脉冲形式，每个脉冲对应轮子转动的一定角度。

# 总结来说，编码器是一种角度传感器，但它们在机器人应用中主要用于通过测量轮子的旋转来计算移动距离和速度。编码器的数据可以转换为线速度和角速度，这对于机器人的定位和导航至关重要。

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
import math
import minimalmodbus
import tf_transformations
import serial
from serial import serialutil
import time


class OdomCalculator(Node):
    def __init__(self, port="/dev/ttyUSB0"):
        super().__init__('odom_diff')
        self.publisher = self.create_publisher(Odometry, '/odom_diff', 10)
        
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # 编码器初始值，实际应从硬件接口获取
        self.l_enc_pos = 0
        self.r_enc_pos = 0
        self.last_l_enc_pos = self.l_enc_pos
        self.last_r_enc_pos = self.r_enc_pos

        # 机器人参数
        self.wheel_radius = 0.27        # 轮子半径，单位：米, # TODO
        self.wheel_separation = 0.56    # 轮距，单位：米
        self.encoder_resolution = 1024  # 编码器分辨率（每圈脉冲数）
        
        self.get_logger().info("正在设置左侧编码器")
        self.l_enc_inst = self.init_encoder(port="/dev/encoder_left")
        
        # 设置串口
        self.get_logger().info("正在设置右侧编码器")
        self.r_enc_inst = self.init_encoder(port="/dev/encoder_right")
        
        # 用来控制打印输出频率的
        self.pub_n, self.last_n, self.last_print_time = 0, 0, time.time()
        
        self.timer = self.create_timer(0.1, self.update_odom)  # 10 Hz
        
        
    def init_encoder(self, port):
        ser = None
        try:
            ser = serial.Serial(port=port, baudrate=9600, bytesize=serialutil.EIGHTBITS, parity=serialutil.PARITY_NONE, stopbits=serialutil.STOPBITS_ONE, timeout=1)
        except Exception as e:
            print(f"串口打开失败:{port}, {e}")
            raise e
        try:
            instrument = minimalmodbus.Instrument(port=ser, slaveaddress=1, mode=minimalmodbus.MODE_RTU, debug = False)
        except Exception as e:
            print(f"Instrument失败:{port}, {e}")
        
        try:
            # 编码器重置零点标志位，此地址写入 1 后，即设置编码器当前位置为零点，当前编码器单圈值读取为 0
            instrument.write_register(0x8, 1, functioncode=0x6)
            time.sleep(0.5)
            return instrument
        except Exception as e:
            print(f"write_register编码器重置零点失败:{port}, {e}")
        
        if ser:
            ser.close()
        
    def get_encoder_dis(self,enc_inst):
        multi = enc_inst.read_registers(registeraddress=0, number_of_registers=2, functioncode=3)
        new_val= int(f"{hex(multi[0])[2:]}{hex(multi[1])[2:]}", 16)
        
        # TODO 处理异常漂移 这个地方逻辑有些问题，没有考虑正常增加到这个位置
        if new_val > 2 ** 30:
            new_val -= 2 ** 31
        return new_val*360/1024
        
    def update_odom(self):
        try:
            self.l_enc_pos = self.get_encoder_dis(self.l_enc_inst)
            self.r_enc_pos = self.get_encoder_dis(self.r_enc_inst)
            
            diff_l = self.l_enc_pos - self.last_l_enc_pos
            diff_r = self.r_enc_pos - self.last_r_enc_pos
            
            # 更新编码器的最后位置
            self.last_l_enc_pos = self.l_enc_pos
            self.last_r_enc_pos = self.r_enc_pos
            
            # 计算每个轮子的位移
            delta_left =  diff_l /360  *  2 * math.pi * self.wheel_radius
            delta_right = diff_r /360  *  2 * math.pi * self.wheel_radius

            # 计算机器人的平均线速度和角速度
            delta_s = (delta_right + delta_left) / 2.0
            delta_theta = (delta_right - delta_left) / self.wheel_separation

            # 更新机器人的位置和姿态
            self.theta += delta_theta
            self.x += delta_s * math.cos(self.theta)
            self.y += delta_s * math.sin(self.theta)

            # 发布里程计数据
            odom_msg = Odometry()
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = 'odom'
            odom_msg.child_frame_id = 'base_link'
            odom_msg.pose.pose.position.x = self.x
            odom_msg.pose.pose.position.y = self.y
            
            # 将欧拉角转换为四元数（roll, pitch, yaw）
            quaternion = tf_transformations.quaternion_from_euler(0.0, 0.0, delta_theta)
            quat_msg = Quaternion()
            quat_msg.x = quaternion[0]
            quat_msg.y = quaternion[1]
            quat_msg.z = quaternion[2]
            quat_msg.w = quaternion[3]
            odom_msg.pose.pose.orientation = quat_msg
            
            self.publisher.publish(odom_msg)
            
            now_time = time.time()
            self.pub_n = self.pub_n + 1
            if now_time - self.last_print_time > 1:
                rate = (self.pub_n - self.last_n)/(now_time - self.last_print_time)
                self.get_logger().info(f"++++++ : {odom_msg.pose.pose.position} rate:{rate}hz l_enc_pos:{self.l_enc_pos} r_enc_pos:{self.r_enc_pos}")
                self.last_print_time = time.time()
                self.last_n = self.pub_n
            
        except Exception as e:
            print(e)

def main(args=None):
    rclpy.init(args=args)
    odom_calculator = OdomCalculator()
    rclpy.spin(odom_calculator)
    odom_calculator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

'''
░░░░░░░░░░░░░░░░░░░░░░░░▄░░
░░░░░░░░░▐█░░░░░░░░░░░▄▀▒▌░
░░░░░░░░▐▀▒█░░░░░░░░▄▀▒▒▒▐░
░░░░░▄▄▀▒░▒▒▒▒▒▒▒▒▒█▒▒▄█▒▐░
░░░▄▀▒▒▒░░░▒▒▒░░░▒▒▒▀██▀▒▌░
░░▐▒▒▒▄▄▒▒▒▒░░░▒▒▒▒▒▒▒▀▄▒▒░
░░▌░░▌█▀▒▒▒▒▒▄▀█▄▒▒▒▒▒▒▒█▒▐
░▐░░░▒▒▒▒▒▒▒▒▌██▀▒▒░░░▒▒▒▀▄
░▌░▒▄██▄▒▒▒▒▒▒▒▒▒░░░░░░▒▒▒▒
▀▒▀▐████▌▄░▀▒▒░░░░░░░░░░▒▒▒
狗狗保佑代码无bug
'''