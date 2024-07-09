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

class InfiniteEncoder:
    def __init__(self,name:str, port:str, wheel_radius:float, single_mode=False):
        
        self.name:str = name
        
        # 这是编码器上一次读取的值，用于判断如何计数
        self._last_enc_value_raw:int = 0
        
        # 这次总计数上一次的值，用于计算偏移
        self.last_value_raw:int = 0
        self.init_value_raw :int= 0
        self.cur_value_raw:int = 0
        
        self.delta_move:float = 0
        self.total_delta_move:float = 0
        self.encoder_range:int = 2**31
        
        self.single_mode = single_mode
        if single_mode:
            self.encoder_range:int = 2**10
        
        self.port:str = port
        self.wheel_radius:float = wheel_radius
        
        # 编码器初始化
        self.last_time = time.time()
        self.instrument = self.init_encoder(self.port)
        
        # 保留历史数据，用于绘图测试
        # self.history = np.array([self._enc_value_raw, self.cur_value_raw, self.delta_move,self.total_delta_move])
    
    def init_encoder(self, port:str) -> minimalmodbus.Instrument :
        ser = None
        try:
            ser = serial.Serial(port=port, baudrate=9600, bytesize=serialutil.EIGHTBITS, parity=serialutil.PARITY_NONE, stopbits=serialutil.STOPBITS_ONE, timeout=1)
        except Exception as e:
            print(f"串口打开失败:{port}, {e}")
            raise e
        try:
            self.instrument = minimalmodbus.Instrument(port=ser, slaveaddress=1, mode=minimalmodbus.MODE_RTU, debug = False)
        except Exception as e:
            print(f"Instrument失败:{port}, {e}")
        try:
            # 编码器重置零点标志位，此地址写入 1 后，即设置编码器当前位置为零点，当前编码器单圈值读取为 0
            self.instrument.write_register(0x8, 1, functioncode=0x6)
            time.sleep(0.5)
            self._enc_value_raw = self.read_raw_value()
            self.cur_value_raw = self.update(_enc_raw=self._enc_value_raw)
            self.last_value_raw = self.cur_value_raw
            self.init_value_raw = self.cur_value_raw
            
        except Exception as e:
            print(f"write_register编码器重置零点失败:{port}, {e}")
            if ser:
                ser.close()
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
        values = self.instrument.read_registers(registeraddress=0, number_of_registers=dl, functioncode=3)
        self._enc_value_raw = self._get_value(values=values)
        return self._enc_value_raw

    def update_status(self):
        now = time.time()
        self._enc_value_raw= self.read_raw_value()             # 直接从编码器读出的值
        self.cur_value_raw = self.update(self._enc_value_raw)  # 处理后的累计值
        
        # 计算偏移距离与时间
        diff_val = self.cur_value - self.last_value        # 本地偏移值
        total_diff_val = self.cur_value - self.init_value   # 总偏移值
        self.elapsed = now - self.last_time                      # 花费时间
        
        # 更新上一次的值，注意不要更新成编码器的
        self.last_value_raw = self.cur_value_raw
        self.last_time = now
        
        # 计算每个轮子的位移
        self.delta_move =  diff_val /360  *  2 * math.pi * self.wheel_radius
        self.total_delta_move = total_diff_val /360  *  2 * math.pi * self.wheel_radius
        
        # 计算轮子的速度
        self.speed = self.delta_move / self.elapsed
        
        return  self.cur_value
    
    def _to_value(self, value_raw:int):
        return value_raw / 1024 * 360
    
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
        return f"[{self.name}-->enc_raw:{_en_str} cur_val:{self.cur_value:7.2f} init:{self.init_value:6.2f} totalmove:{self.total_delta_move:6.2f}]"
    
class OdomCalculator(Node):
    def __init__(self):
        super().__init__(node_name='odom_diff')
        self.publisher = self.create_publisher(Odometry, '/odom_diff', 10)
        
        # 总的累积里程
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # 机器人参数
        self.wheel_radius = 0.27        # 轮子半径，单位：米
        self.wheel_separation = 0.56    # 轮距，单位：米
        self.encoder_resolution = 1024  # 编码器分辨率（每圈脉冲数）
        self.encoder_range = 2**31
        self.single_mode = True
        
        # 编码器数值处理类，用于无限计数
        self.get_logger().info("正在设置左侧编码器")
        self.L = InfiniteEncoder(name="左",port="/dev/encoder_left", wheel_radius=self.wheel_radius,single_mode=self.single_mode)
        
        self.get_logger().info("正在设置右侧编码器")
        self.R = InfiniteEncoder(name="右",port="/dev/encoder_right", wheel_radius=self.wheel_radius,  single_mode=self.single_mode)
        
        # 用来控制打印输出频率的
        self.pub_n, self.last_n, self.last_print_time = 0, 0, time.time()
        
        # 循环更新里程计
        self.create_timer(0.1, self.update_odom)
        
    def update_odom(self):
        # while rclpy.ok():
        try:
            # TODO 这个地方实际上由于串口延时，耗费时间略有不同
            self.L.update_status()
            self.R.update_status()

            # 计算差值变化
            delta_s = (self.R.delta_move + self.L.delta_move) / 2.0 
            delta_theta = (self.R.delta_move - self.L.delta_move) / self.wheel_separation
            total_delta_theta = (self.R.total_delta_move - self.L.total_delta_move) / self.wheel_separation
            elapsed = self.L.elapsed # 时间变化
            
            # 更新机器人速度
            self.dx = delta_s / elapsed      # 线速度
            self.dr = delta_theta / elapsed  # 角速度
            
            # 更新机器人坐标及角度
            if (delta_s != 0):
                # calculate distance traveled in x and y
                x = math.cos( delta_theta ) * delta_s
                y = - math.sin( delta_theta ) * delta_s
                # calculate the final position of the robot
                self.x = self.x + ( math.cos( self.theta ) * x - math.sin( self.theta ) * y )
                self.y = self.y + ( math.sin( self.theta ) * x + math.cos( self.theta ) * y )
            # 角度
            # if( delta_theta != 0):
                # self.theta += (delta_right - delta_left) / self.wheel_separation
            self.theta = total_delta_theta %  (2 *math.pi)
            if total_delta_theta < 0:
                self.theta = (total_delta_theta + 2*math.pi) % ( 2 * math.pi)

            # 发布里程计数据
            odom_msg = Odometry()
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = 'odom'
            odom_msg.child_frame_id = 'base_link'
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
                self.get_logger().info(f"X:{self.x:6.2f} Y:{self.y:6.2f} theta={self.theta/math.pi:6.2f}pi {self.L}{self.R} rate:{_rate:6.2f}hz")
                self.last_print_time = time.time()
                self.last_n = self.pub_n
            
        except Exception as e:
            print(e)
            traceback.print_exc()
            
        finally:
            # 执行一些操作
            # self.rate.sleep()
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