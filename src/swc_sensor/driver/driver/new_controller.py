import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import struct
import math
import time

class SerialVelocitySender(Node):
    def __init__(self):
        super().__init__('serial_velocity_sender')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10)
        self.ser = serial.Serial('/dev/driver', 115200, timeout=1)

        self.last_cmd_time = time.time()
        # self.zero_sent = False  # 标记是否已经发送过零速度，避免重复发送
        self.create_timer(0.1, self.timer_callback)  # 100ms检查一次

    def listener_callback(self, msg: Twist):
        linear_mps = msg.linear.x
        angular_rps = msg.angular.z

        # 单位转换
        linear_cmps = linear_mps * 100.0              # m/s -> cm/s
        angular_dps = angular_rps * 180.0 / math.pi   # rad/s -> deg/s

        self.send_control_command(linear_cmps, angular_dps)

        self.last_cmd_time = time.time()
        # self.zero_sent = False  # 收到正常指令，重置标记

    def timer_callback(self):
        elapsed = time.time() - self.last_cmd_time
        if elapsed > 0.5: #  and not self.zero_sent:
            # 超过0.5秒没收到指令，发送零速度指令
            self.send_control_command(0.0, 0.0)
            # self.zero_sent = True

    def calculate_checksum(self, data_bytes):
        checksum = sum(data_bytes) & 0xFF
        return checksum.to_bytes(1, byteorder='little')

    def build_control_frame(self, linear_vel, angular_vel):
        linear_bytes = struct.pack('<f', linear_vel)
        angular_bytes = struct.pack('<f', angular_vel)
        data_bytes = linear_bytes + angular_bytes
        frame = b'\xAA' + data_bytes
        checksum_byte = self.calculate_checksum(frame)
        return frame + checksum_byte

    def send_control_command(self, linear_vel, angular_vel):
        frame = self.build_control_frame(linear_vel, angular_vel)
        self.ser.write(frame)
        self.get_logger().info(f"发送指令：线速度={linear_vel:.2f} cm/s, 角速度={angular_vel:.2f} deg/s")
        self.get_logger().info("发送字节：" + ' '.join(f'{b:02X}' for b in frame))

    def destroy_node(self):
        self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SerialVelocitySender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
