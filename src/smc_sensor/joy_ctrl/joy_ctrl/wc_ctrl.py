#!/usr/bin/env python
# encoding: utf-8

#public lib
import os
import time
import getpass
import threading
from time import sleep

#ros lib
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from actionlib_msgs.msg import GoalID
from std_msgs.msg import Int32, Bool
from std_msgs.msg import String

class JoyTeleop(Node):
    def __init__(self,name):
        super().__init__(name)
        self.joy_active = False
        self.cancel_time = time.time()
        # self.x_speeds = [0.05,0.07,0.1]
        # self.angular_speeds = [0.11,0.13,0.15]

        # 对应国康轮椅的速度，把速度增大一些
        self.x_speeds = [0.2,0.4,0.6, 0.8, 1.0]
        self.angular_speeds = [0.12,0.16,0.2, 0.2, 0.2]

        self.speed_index = 0
        
        self.xspeed_info = {"min":0.2, "max":1.6, "diff":0.2}
        self.angular_speed_info = {"min":0.12, "max":0.2, "diff":0.02}
        
        #create pub
        self.pub_cmdVel = self.create_publisher(Twist,'cmd_vel',  10)
        self.pub_odom2init = self.create_publisher(String,'odom2init',  10)
        #create sub
        self.sub_Joy = self.create_subscription(Joy,'joy', self.buttonCallback, 10)
        
        #declare parameter and get the value
        self.xspeed_limit = self.x_speeds[self.speed_index]
        self.angular_speed_limit = self.angular_speeds[self.speed_index]

        # 上次换挡的时间
        self.last_swith_gear_time = time.time()

        self.last_print_time = time.time()
        
        self.last_init_odom = time.time()
        
    
    def buttonCallback(self,joy_data):
        if not isinstance(joy_data, Joy): 
            return
        self.user_jetson(joy_data)
        
    def user_jetson(self, joy_data:Joy):
        curtime = time.time()
        
        # Y is unlock, and X is lock
        if  1 in joy_data.buttons[3:5] and curtime - self.cancel_time > 0.3:
            self.joy_active = False
            if joy_data.buttons[4] == 1:
                self.joy_active = True
            if joy_data.buttons[3] == 1:
                self.joy_active = False
            if self.joy_active:
                self.get_logger().info("joy control enabled!")
            else:
                self.get_logger().info("joy control disabled!")
            
            for i in range(3):
                self.pub_cmdVel.publish(Twist())
            self.cancel_time = curtime
        
        if joy_data.buttons[14] == 1 and curtime - self.last_init_odom > 1:
            msg = String()                                            # 创建一个String类型的消息对象
            msg.data = 'reset odom to init'                                  # 填充消息对象中的消息数据
            self.pub_odom2init.publish(msg)
            self.get_logger().info("send set odom msg") 
            self.last_init_odom = curtime
        
        
        #linear Gear control
        if 1 in joy_data.buttons[0:2] and curtime - self.last_swith_gear_time > 0.3:
            
            add_sign = 1
            if joy_data.buttons[0] == 1:
                add_sign = -1
            elif joy_data.buttons[1] == 1:
                add_sign = 1
            
            s_index = self.speed_index + add_sign
            
            if s_index > len(self.x_speeds)-1:
                s_index = len(self.x_speeds) -1
            elif s_index < 0:
                s_index = 0
            
            self.speed_index = s_index
            
            self.xspeed_limit = self.x_speeds[self.speed_index]
            self.angular_speed_limit = self.angular_speeds[self.speed_index]
            self.last_swith_gear_time = curtime
            self.get_logger().info(f"cur_gear is: {self.speed_index}  {self.xspeed_limit}  {self.angular_speed_limit}")
        
        # 上下按钮为joy_data.axes[7]，左右按钮为joy_data.axes[6]，取值为正负1
        # x_xishu = joy_data.axes[7] if joy_data.axes[7]!=0 else joy_data.axes[1]
        # angle_xishu = joy_data.axes[6] if joy_data.axes[6]!=0 else joy_data.axes[0]
        
        x_xishu = joy_data.axes[1] if joy_data.axes[1]!=0 else joy_data.axes[7]
        angle_xishu = joy_data.axes[0] if joy_data.axes[0]!=0 else joy_data.axes[6]
        
        
        # joy_data.axes[1]是接收左边操纵杆前后拨动的信号
        xlinear_speed = self.filter_data(x_xishu) * self.xspeed_limit

        # joy_data.axes[0]是接收左边操纵杆左右拨动的信号
        angular_speed = self.filter_data(angle_xishu) * self.angular_speed_limit

        twist = Twist()
        twist.linear.x = xlinear_speed
        twist.angular.z = angular_speed
        
        if self.joy_active:
            for i in range(3): 
                self.pub_cmdVel.publish(twist)

        # logging to console 
        if curtime - self.last_print_time > 1:
            # self.get_logger().info(f"{xlinear_speed,angular_speed, joy_data.buttons, joy_data.axes}")
            self.last_print_time = curtime 
        
    def filter_data(self, value):
        if abs(value) < 0.2: value = 0
        return value
            
def main():
    rclpy.init()
    joy_ctrl = JoyTeleop('joy_ctrl')
    rclpy.spin(joy_ctrl)        

if __name__ == "__main__":
    main()