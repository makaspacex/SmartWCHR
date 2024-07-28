from re import L
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import math
import time
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from rclpy.qos import qos_profile_sensor_data

def sign(x):
    if x > 0:
        return 1
    elif x < 0:
        return -1
    else:
        return 0

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
            desired_change = (
                max_speed_change if desired_change > 0 else -max_speed_change
            )

        # 更新速度
        self.current_speed += desired_change
        return self.current_speed


class ControllerNode(Node):
    """控制节点"""

    def __init__(self, portname="/dev/driver", rate=115200):
        super().__init__("driver")

        self.intervalWheel = 0.570  # 左右轮间距（m）
        self.FrameHead_Macro = 0x42  # 帧头
        self.SpeedControll_Macro = 0x01  # 速度控制地址
        self.MAX_SPEED = 0.5  # 最大速度
        self.buffer = 12 * [0]  # 串口发送内容

        self.liner_x_range = [0.03, 0.07]
        self.angle_z_range = [0.10, 0.15]

        self.ser = None
        try:
            # 创建接口对象 串口设备名：'/dev/driver' 比特率：115200
            self.ser = serial.Serial(portname, rate, timeout=1000)
        except Exception as e:
            print("串口异常：", e)
            raise e
        self.last_print_time = time.time()
        self.update_buffer(linear_x=0, angle_z=0)

        # 接收cmd数据、计算发送数据
        self.cmd_sub_ = self.create_subscription(
            Twist, "cmd_vel", self.CMDCallBack, QoSProfile(history=QoSHistoryPolicy.KEEP_LAST,reliability=QoSReliabilityPolicy.BEST_EFFORT)
        )
        # 每隔0.05s向串口发送buffer
        self.ser_write_timer_ = self.create_timer(0.1, self.SpeedCallBack)
        self.get_logger().info("Hi from controller")

        # # 当前状态   stop, starting, running, stopping四种
        # self.state = 'stop'
        # self.left_Control = SpeedController()
        # self.right_Control = SpeedController()

    def CMDCallBack(self, msg: Twist):
        linear_x, angle_z = msg.linear.x, msg.angular.z
        self.update_buffer(linear_x=linear_x, angle_z=angle_z)

    def SpeedCallBack(self):
        if not self.ser:
            return
        self.ser.write(self.buffer)

    def get_buffer(self, sendData, len=12):
        check = 0x0

        check += self.FrameHead_Macro
        check += self.SpeedControll_Macro
        check += len
        for data in sendData:
            check += data

        buffer = 12 * [0]
        buffer[0] = self.FrameHead_Macro
        buffer[1] = self.SpeedControll_Macro
        buffer[2] = len
        buffer[3 : 3 + 8] = sendData
        buffer[len - 1] = check & 0b11111111
        return buffer

    def update_buffer(self, linear_x, angle_z, max_x=0.1, max_z=0.1):

        _linear_x, _angle_z = linear_x, angle_z
        if abs(linear_x) >= max(*self.liner_x_range):
            linear_x = sign(linear_x) * self.liner_x_range[1]
        elif abs(linear_x) >= min(*self.liner_x_range) and abs(linear_x) < max(
            *self.liner_x_range
        ):
            pass
        elif abs(linear_x) >= 0.022 and abs(linear_x) < min(*self.liner_x_range):
            linear_x = sign(linear_x) * self.liner_x_range[0]
        else:
            linear_x = 0

        if abs(angle_z) >= max(*self.angle_z_range):
            angle_z = sign(angle_z) * self.angle_z_range[1]
        elif abs(angle_z) >= min(*self.angle_z_range) and abs(angle_z) < max(
            *self.angle_z_range
        ):
            pass
        elif abs(angle_z) >= 0.08 and abs(angle_z) < min(*self.angle_z_range):
            angle_z = sign(angle_z) * self.angle_z_range[0]
        else:
            angle_z = 0

        # v = (Vr + Vl) / 2 线速度公式
        # w = (Vr - Vl) / l 角速度公式
        l_speed = linear_x - 0.5 * angle_z * self.intervalWheel
        r_speed = linear_x + 0.5 * angle_z * self.intervalWheel
        l_speed *= 1.12

        # 处理最大速度
        l_speed = min(l_speed, self.MAX_SPEED)
        r_speed = min(r_speed, self.MAX_SPEED)

        # 处理最小值
        # if l_speed < self.MIN_SPEED:
        #     l_speed = 0

        curtime = time.time()
        if curtime - self.last_print_time > 1:
            self.last_print_time = curtime
            print(l_speed, r_speed)
            self.get_logger().info(
                f"l_speed:{l_speed:.2f} r_speed:{r_speed:.2f} linearx:{_linear_x} {linear_x}  angle_z:{_angle_z} {angle_z}"
            )

        sendData = 8 * [0]
        l_speed = int((l_speed / self.MAX_SPEED) * 1500).to_bytes(
            4, byteorder="big", signed=True
        )
        r_speed = int((r_speed / self.MAX_SPEED) * 1500).to_bytes(
            4, byteorder="big", signed=True
        )
        sendData[0:4] = l_speed
        sendData[4:] = r_speed
        self.buffer = self.get_buffer(sendData=sendData)


def main(args=None):
    rclpy.init(args=args)
    controller = ControllerNode()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
