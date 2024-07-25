from re import L
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from threading import Thread


class vel_subscriber_Node(Node):
    """控制节点"""
    def __init__(self):
        super().__init__('vel_subscriber')
        self.xdata, self.ydata1, self.ydata2, self.count = [], [], [],0
        self.cmd_sub_ = self.create_subscription(Twist, 'cmd_vel', self.CMDCallBack, 10)
        
        self.plot_thread = Thread(target=self.plot_real_time_subplots, daemon=True)
        self.plot_thread.start()
        
    
    def CMDCallBack(self, msg: Twist):
        linear_x, angle_z =  msg.linear.x, msg.angular.z

        self.xdata.append(self.count)
        self.ydata1.append(linear_x)
        self.ydata2.append(angle_z)
        self.count += 1
        self.get_logger().info(f'Received cmd_vel data: linear_x: {linear_x}, angle_z: {angle_z}')
        

    def plot_real_time_subplots(self, xlabel='Index', ylabel1='Value1', ylabel2='Value2', title1='Real-Time Data Plot 1', title2='Real-Time Data Plot 2', interval=1000, window_size=1000):
        # 创建两个子图
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 10))
        
        ln1, = ax1.plot([], [], 'b-', animated=True, label=ylabel1)
        ln2, = ax2.plot([], [], 'r-', animated=True, label=ylabel2)

        def init():
            ax1.set_xlim(0, window_size)
            ax1.set_ylim(0, 1)
            ax1.set_xlabel(xlabel)
            ax1.set_ylabel(ylabel1)
            ax1.set_title(title1)
            ax1.legend()

            ax2.set_xlim(0, window_size)
            ax2.set_ylim(0, 1)
            ax2.set_xlabel(xlabel)
            ax2.set_ylabel(ylabel2)
            ax2.set_title(title2)
            ax2.legend()
            
            return ln1, ln2

        def update(frame):
            count = len(self.xdata)
            
            if len(self.xdata) == 0:
                return ln1, ln2
            
            ln1.set_data(self.xdata[-window_size:], self.ydata1[-window_size:])
            ln2.set_data(self.xdata[-window_size:], self.ydata2[-window_size:])

            ax1.set_xlim(max(0, count - window_size), count)
            ax2.set_xlim(max(0, count - window_size), count)

            ax1.set_ylim(min(self.ydata1[-window_size:]) - 0.1, max(self.ydata1[-window_size:]) + 0.1)
            ax2.set_ylim(min(self.ydata2[-window_size:]) - 0.1, max(self.ydata2[-window_size:]) + 0.1)
            
            return ln1, ln2

        ani = FuncAnimation(fig, update, init_func=init, blit=True, interval=interval)
        plt.tight_layout()
        # plt.savefig('real_time_plot.png')
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    odom_calculator = vel_subscriber_Node()
    rclpy.spin(odom_calculator)
    odom_calculator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

