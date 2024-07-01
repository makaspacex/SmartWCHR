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
from geometry_msgs.msg import Quaternion
import math

class OdomCalculator(Node):
    def __init__(self):
        super().__init__('odom_calculator')
        self.publisher = self.create_publisher(Odometry, '/odom', 10)
        self.timer = self.create_timer(0.1, self.update_odom)  # 10 Hz
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # 假设的编码器读数，实际应从硬件接口获取
        self.left_encoder = 0
        self.right_encoder = 0
        self.last_left_encoder = 0
        self.last_right_encoder = 0

        # 机器人参数
        self.wheel_radius = 0.1  # 轮子半径，单位：米
        self.wheel_separation = 0.5  # 轮距，单位：米
        self.encoder_resolution = 360  # 编码器分辨率（每圈脉冲数）

    def update_odom(self):
        # 计算每个轮子的位移
        delta_left = (self.left_encoder - self.last_left_encoder) * (2 * math.pi * self.wheel_radius) / self.encoder_resolution
        delta_right = (self.right_encoder - self.last_right_encoder) * (2 * math.pi * self.wheel_radius) / self.encoder_resolution

        # 更新编码器的最后位置
        self.last_left_encoder = self.left_encoder
        self.last_right_encoder = self.right_encoder

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
        odom_msg.pose.pose.orientation = Quaternion(*self.euler_to_quaternion(0, 0, self.theta))
        self.publisher.publish(odom_msg)

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

def main(args=None):
    rclpy.init(args=args)
    odom_calculator = OdomCalculator()
    rclpy.spin(odom_calculator)
    odom_calculator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()