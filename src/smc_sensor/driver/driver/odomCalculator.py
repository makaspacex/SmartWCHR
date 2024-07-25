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
import rclpy.clock
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
import math
import minimalmodbus
import rclpy.time
import rclpy.timer
import tf_transformations
import serial
from serial import serialutil
import time
import traceback
import numpy as np
from std_msgs.msg import String
from collections import deque
from threading import Thread
from dataclasses import dataclass
from threading import Lock
from copy import deepcopy
import threading


@dataclass
class EncoderStatus:
    now:float
    cur_value:float
    diff_val:float
    total_diff_val:float
    elapsed:float
    delta_move:float
    total_delta_move:float
    speed:float

class InfiniteEncoder:
    def __init__(self,name:str, port:str, wheel_radius:float, slaveaddress=1, baudrate=9600, single_mode=False, wheel_encoder_ratio=1.0):
        
        self.name:str = name
        
        # 从地址。多个编码器可以接一根线，依靠从地址区分
        self.slaveaddress = slaveaddress
        
        # 这是编码器上一次读取的值，用于判断如何计数
        self._last_enc_value_raw:int = 0
        
        # 这次总计数上一次的值，用于计算偏移
        self.last_value_raw:int = 0
        self.init_value_raw :int= 0
        self.cur_value_raw:int = 0
        
        self.delta_move:float = 0
        self.total_delta_move:float = 0
        self.encoder_range:int = 2**31
        
        self.baudrate = baudrate
        
        self.single_mode = single_mode
        if single_mode:
            self.encoder_range:int = 2**10
        
        self.port:str = port
        self.wheel_radius:float = wheel_radius

        self.wheel_encoder_ratio = wheel_encoder_ratio
        
        self.ser = None
        # 编码器初始化
        self.last_time = time.time()
        self.instrument = self.init_encoder()
        
        # 历史过去一段时间的数据
        self.status_his = deque(maxlen=20)
        
        # 保留历史数据，用于绘图测试
        # self.history = np.array([self._enc_value_raw, self.cur_value_raw, self.delta_move,self.total_delta_move])
    
    def init_to_zero(self):
        try:
            # 编码器重置零点标志位，此地址写入 1 后，即设置编码器当前位置为零点，当前编码器单圈值读取为 0
            self.instrument.write_register(0x8, 1, functioncode=0x6)
            time.sleep(0.5)
            self._enc_value_raw = self.read_raw_value()
            self.cur_value_raw = self.update(_enc_raw=self._enc_value_raw)
            self.last_value_raw = self.cur_value_raw
            self.init_value_raw = self.cur_value_raw
            self.status_his = deque(maxlen=20)
        except Exception as e:
            print(f"write_register编码器重置零点失败: {e}")
            if self.ser and self.ser.is_open:
                self.ser.close()
        
    def init_encoder(self) -> minimalmodbus.Instrument :
        try:
            self.ser = serial.Serial(port=self.port, baudrate=self.baudrate, bytesize=serialutil.EIGHTBITS, parity=serialutil.PARITY_NONE, stopbits=serialutil.STOPBITS_ONE, timeout=1)
        except Exception as e:
            print(f"串口打开失败:{self.port}, {e}")
            raise e
        try:
            self.instrument = minimalmodbus.Instrument(port=self.ser, slaveaddress=self.slaveaddress, mode=minimalmodbus.MODE_RTU, debug = False)
        except Exception as e:
            print(f"Instrument失败:{self.port}, {e}")
        
        try:
            self._enc_value_raw = self.read_raw_value()
            self.cur_value_raw = self.update(_enc_raw=self._enc_value_raw)
            self.last_value_raw = self.cur_value_raw
            self.init_value_raw = self.cur_value_raw
        except Exception as e:
            raise e
        
        return self.instrument

    def update(self, _enc_raw:int) -> int:
        
        #!!! 如果编码器值当前值远远大于上一次值，说明是从零点反转了
        #!!! 因为正转不可能这么快
        
        # 计算差值，正常情况下
        delta = _enc_raw - self._last_enc_value_raw
        
        # 处理过零点的异常情况
        # 差值大于一半的范围 --> 后转了
        if delta > 0 and delta > self.encoder_range / 2:
            delta = - (self.encoder_range - _enc_raw + self._last_enc_value_raw)
        # 差值小于一半的范围 --> 后转了
        elif delta < 0 and delta < -self.encoder_range / 2:
            delta = self.encoder_range - self._last_enc_value_raw + _enc_raw
        
        # 更新无限计数
        self.cur_value_raw += int(delta)
        
        # 更新上一次编码器的值，注意，不要更新成实际的值
        self._last_enc_value_raw = _enc_raw
        
        # 返回无限计数的当前值
        return self.cur_value_raw

    def _get_value(self, values):
        if len(values) == 1:
            return int(values[0])
        new_val= int(f"{hex(values[0])[2:]}{hex(values[1])[2:]}", 16)
        return new_val

    def read_raw_value(self) -> int:
        dl = 2
        if self.single_mode:
            dl = 1
        self._enc_value_raw = self.instrument.read_long(registeraddress=0, number_of_registers=dl, functioncode=3)
        return self._enc_value_raw

    def update_status(self):
        self._enc_value_raw= self.read_raw_value()             # 直接从编码器读出的值
        now = time.time()
        self.cur_value_raw = self.update(self._enc_value_raw)  # 处理后的累计值

        # 计算偏移距离与时间
        diff_val = self.cur_value - self.last_value        # 本次偏移值
        total_diff_val = self.cur_value - self.init_value   # 总偏移值
        self.elapsed = now - self.last_time                      # 花费时间
        
        # 更新上一次的值，注意不要更新成编码器的
        self.last_value_raw = self.cur_value_raw
        self.last_time = now
        
        # 计算每个轮子的位移
        self.delta_move =  diff_val /360  *  2 * math.pi * self.wheel_radius / self.wheel_encoder_ratio
        self.total_delta_move = total_diff_val /360  *  2 * math.pi * self.wheel_radius / self.wheel_encoder_ratio
        
        self.total_delta_move = round(self.total_delta_move, 4)
        
        # 计算轮子的速度
        self.speed = self.delta_move / self.elapsed
        
        
        # 保存历史状态
        s_data= {"now":now, 
                  "cur_value":self.cur_value, 
                  "diff_val":diff_val,
                  "total_diff_val":total_diff_val, 
                  "elapsed": self.elapsed,
                  "delta_move":self.delta_move ,
                  "total_delta_move":self.total_delta_move,
                  "speed":self.speed}
        
        enc_status = EncoderStatus(**s_data)
        self.status_his.append(enc_status)
        
        return  self.cur_value
    
    def _to_value(self, value_raw:int):
        return value_raw / 1024 * 360

    def __del__(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    @property
    def last_value(self):
        return self._to_value(self.last_value_raw)
    @property
    def init_value(self):
        return self._to_value(self.init_value_raw)
    
    @property
    def cur_value(self):
        return self._to_value(self.cur_value_raw)
    
    def __str__(self):
        _en_str = f"{self._enc_value_raw:10d}"
        if self.single_mode:
            _en_str = f"{self._enc_value_raw:4d}"
        return f"[{self.name}-->enc_raw:{_en_str} cur_val:{self.cur_value:10.2f} init:{self.init_value:6.2f} totalmove:{self.total_delta_move:6.2f}]"
    
class OdomCalculator(Node):
    def __init__(self):
        super().__init__(node_name='odom_cal_node')
        self.publisher = self.create_publisher(Odometry, '/odom_diff', 10)
        self.sub_odom2init = self.create_subscription(String, '/odom2init', self.reset_odom, 10)
        
        self.declare_parameter("robot_name", value="gk01")
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        
        # 总的累积里程
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # 机器人参数
        self.wheel_radius_l,self.wheel_radius_r  = 0.27, 0.27        # 轮子半径，单位：米
        self.wheel_separation = 0.56    # 轮距，单位：米
        self.single_mode = False        # 是否采用单圈模式
        left_slaveaddress, right_slaveaddress = 1,1 # 轮速比
        self.wheel_encoder_ratio_l,self.wheel_encoder_ratio_r = 1, 1
        if self.robot_name == 'gk01':
            self.wheel_radius_l,self.wheel_radius_r = 0.172, 0.172
            # 轮子半径，单位：米，轮子半径为1是，实际行走距离为10.127时，左轮59.19，右轮58.76
            # self.wheel_separation = 0.595    # 轮距，单位：米
            # self.wheel_separation = 0.5831163056596582    # 轮距，单位：米
            self.wheel_separation = 0.6    # 轮距，单位：米
            self.single_mode = False
            self.wheel_encoder_ratio_l,self.wheel_encoder_ratio_r = 29.5, 29.5
            left_slaveaddress, right_slaveaddress = 1,2
        
        self.get_logger().info(f"robot_name is {self.robot_name}")
        
        self.baudrate = 9600
        
        self.get_logger().info("正在设置左侧编码器")
        self.L = InfiniteEncoder(name="左",port="/dev/encoder_left",slaveaddress = left_slaveaddress, baudrate=self.baudrate, wheel_encoder_ratio=self.wheel_encoder_ratio_l, wheel_radius=self.wheel_radius_l, single_mode=self.single_mode)

        self.get_logger().info("正在设置右侧编码器")
        self.R = InfiniteEncoder(name="右",port="/dev/encoder_right",slaveaddress =right_slaveaddress, baudrate=self.baudrate, wheel_encoder_ratio=self.wheel_encoder_ratio_r, wheel_radius=self.wheel_radius_r, single_mode=self.single_mode)
        
        
        
        # 用来控制打印输出频率的
        self.pub_n, self.last_n, self.last_print_time = 0, 0, time.time()
        
        # 不断更新编码器状态
        self.read_enc_event = threading.Event()
        self.L_finish_event,self.R_finish_event = threading.Event(), threading.Event()
        
        Thread(target=self.update_enc_status, daemon=True, args=(self.L,self.L_finish_event,)).start()
        Thread(target=self.update_enc_status, daemon=True, args=(self.R,self.R_finish_event,)).start()
        
        # 循环更新里程计
        self.create_timer(0.03, self.update_odom)
        
        # 使用事件是因为线程锁的频繁获取会对速度造成较大影响
        self.reset_odom_event = threading.Event()
        
        # 最终的编码器状态
        self.r_status = None
        self.l_status = None
        
        self.enc_update_lock = Lock()
        
        # 重置起始点为0
        self.reset_odom(None)
        
        
    def reset_odom(self, msg):
        # 清除reset事件并且等待左右两个编码器完成读数
        self.reset_odom_event.clear()
        # 总的累积里程
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.L.init_to_zero()
        self.R.init_to_zero()
        self.reset_odom_event.set()
        self.get_logger().info("reset odom to zeto")
    
    def update_enc_status(self, encoder:InfiniteEncoder, finish:threading.Event):
        while True:
            try:
                self.reset_odom_event.wait()
                self.read_enc_event.wait()
                encoder.update_status()
            except Exception:
                pass
            finally:
                finish.set()
    
    def update_enc_status2(self):
        while True:
            try:
                self.L.update_status()
                self.R.update_status()
                
                l_status:EncoderStatus = self.L.status_his[-1]
                
                r_status:EncoderStatus = self.R.status_his[-1]
                
                min_delta_t = math.inf
                for ele in list(self.R.status_his)[::-1]:
                    _r_status:EncoderStatus = ele
                    if abs(_r_status.now - l_status.now) < min_delta_t:
                        r_status = _r_status
                        min_delta_t = abs(_r_status.now - l_status.now)
                
                with self.enc_update_lock:
                    # 更新到编码器的全局状态
                    self.r_status = r_status
                    self.l_status = l_status
                
            except Exception:
                pass
        # 寻找时间最接近的编码器数据
        
    def solution_1(self):
        # 计算差值变化
        delta_s = (self.R.delta_move + self.L.delta_move) / 2.0 
        delta_theta = (self.R.delta_move - self.L.delta_move) / self.wheel_separation
        total_delta_theta = (self.R.total_delta_move - self.L.total_delta_move) / self.wheel_separation
        elapsed = self.L.elapsed # 时间变化
        
        # 更新机器人速度
        self.dx = delta_s / elapsed      # 线速度
        self.dr = delta_theta / elapsed  # 角速度
        
        # 更新机器人坐标及角度
        delta_x = math.cos( delta_theta ) * delta_s
        delta_y = - math.sin( delta_theta ) * delta_s
        
        # calculate the final position of the robot
        self.x += math.cos( self.theta ) * delta_x - math.sin( self.theta ) * delta_y 
        self.y += math.sin( self.theta ) * delta_x + math.cos( self.theta ) * delta_y 
        
        # self.x += delta_x
        # self.y += delta_y
        
        # 角度
        # self.theta += delta_theta
        self.theta = total_delta_theta
        
        if self.theta > 0:
            self.theta = self.theta % (2 *math.pi)
        else:
            self.theta = self.theta % (- 2 * math.pi)
            #######################################################################################################################

    def solution2(self):
        if math.isclose(self.L.delta_move, self.R.delta_move):
            delta_x = self.L.delta_move * math.cos(self.theta)
            delta_y = self.L.delta_move * math.sin(self.theta)
            delta_theta = 0
        else:
            delta_theta = (self.R.delta_move - self.L.delta_move) / self.wheel_separation
            R = self.wheel_separation / 2 * (self.R.delta_move + self.L.delta_move) / (self.R.delta_move - self.L.delta_move)
            delta_x = R * (math.sin(self.theta + delta_theta) - math.sin(self.theta))
            delta_y = R * (math.cos(self.theta) - math.cos(self.theta + delta_theta))
        elapsed = self.L.elapsed # 时间变化
        delta_s = (self.R.delta_move + self.L.delta_move) / 2.0 
        # 更新机器人速度
        self.dx = delta_s / elapsed      # 线速度
        self.dr = delta_theta / elapsed  # 角速度
        
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
    
    def solution3(self):
        total_delta_theta = (self.R.total_delta_move - self.L.total_delta_move) / self.wheel_separation
        delta_theta = total_delta_theta - self.theta
        # self.get_logger().info(f"{abs(self.L.delta_move - self.R.delta_move):12.6f}")
        if abs(self.L.delta_move - self.R.delta_move)<0.01:
            delta_x = self.L.delta_move * math.cos(self.theta)
            delta_y = self.L.delta_move * math.sin(self.theta)
        else:
            R = self.wheel_separation / 2 * (self.R.delta_move + self.L.delta_move) / (self.R.delta_move - self.L.delta_move)
            delta_x = R * (math.sin(self.theta + delta_theta) - math.sin(self.theta))
            delta_y = R * (math.cos(self.theta) - math.cos(self.theta + delta_theta))
        elapsed = self.L.elapsed # 时间变化
        delta_s = (self.R.delta_move + self.L.delta_move) / 2.0 
        # 更新机器人速度
        self.dx = delta_s / elapsed      # 线速度
        self.dr = delta_theta / elapsed  # 角速度
        
        self.x += delta_x
        self.y += delta_y
        self.theta = total_delta_theta

    def update_odom(self):
        try:
            # self.solution2()
            # self.L.update_status()
            # self.R.update_status()
            
            self.L_finish_event.clear()
            self.R_finish_event.clear()
            self.read_enc_event.set()
            self.read_enc_event.clear()
            self.L_finish_event.wait()
            self.R_finish_event.wait()
            
            self.solution3()
            
            # 发布里程计数据
            odom_msg = Odometry()
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = 'odom'
            odom_msg.child_frame_id = 'base_footprint'
            odom_msg.pose.pose.position.x = self.x
            odom_msg.pose.pose.position.y = self.y
            odom_msg.twist.twist.linear.x = self.dx
            odom_msg.twist.twist.linear.y = 0.0
            odom_msg.twist.twist.angular.z = self.dr
            
            # 将欧拉角转换为四元数（roll, pitch, yaw）
            quaternion = tf_transformations.quaternion_from_euler(0.0, 0.0, self.theta)
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
                _rate = (self.pub_n - self.last_n)/(now_time - self.last_print_time)
                self.get_logger().info(f"X:{self.x:8.2f} Y:{self.y:8.2f} theta={self.theta/math.pi * 180 :6.2f}C {self.L}{self.R} rate:{_rate:6.2f}hz")
                self.last_print_time = time.time()
                self.last_n = self.pub_n
            
        except Exception as e:
            pass
        

def main(args=None):
    rclpy.init(args=args)
    odom_calculator = OdomCalculator()
    rclpy.spin(odom_calculator)
    odom_calculator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''
                  _ooOoo_
                 o8888888o
                 88" . "88
                 (| -_- |)
                 O\  =  /O
              ____/`---'\____
            .'  \\|     |//  `.
           /  \\|||  :  |||//  \
          /  _||||| -:- |||||-  \
          |   | \\\  -  /// |   |
        | \_|  ''\---/''  |   |
         \  .-\__  `-`  ___/-. /
       ___`. .'  /--.--\  `. . __
    ."" '<  `.___\_<|>_/___.'  >'"".
   | | :  `- \`.;`\ _ /`;.`/ - ` : | |
     \  \ `-.   \_ __\ /__ _/   .-` /  /
 =====`-.____`-.___\_____/___.-`____.-'======
                  `=---='
 ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
              佛祖保佑         永无BUG
'''