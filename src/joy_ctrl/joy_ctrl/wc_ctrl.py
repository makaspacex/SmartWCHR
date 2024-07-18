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


class JoyTeleop(Node):
    def __init__(self,name):
        super().__init__(name)
        self.Joy_active = False
        self.Buzzer_active = False
        self.RGBLight_index = 0
        self.cancel_time = time.time()
        self.user_name = getpass.getuser()

        # self.x_speeds = [0.05,0.07,0.1]
        # self.angular_speeds = [0.11,0.13,0.15]

        # 对应国康轮椅的速度，把速度增大一些
        self.x_speeds = [0.15,0.18,0.20]
        self.angular_speeds = [0.18,0.22,0.24]



        self.speed_index = 0
        
        #create pub
        self.pub_cmdVel = self.create_publisher(Twist,'cmd_vel',  10)
        self.pub_JoyState = self.create_publisher(Bool,"JoyState",  10)
        #create sub
        self.sub_Joy = self.create_subscription(Joy,'joy', self.buttonCallback, 10)
        
        #declare parameter and get the value
        self.xspeed_limit = self.x_speeds[self.speed_index]
        self.angular_speed_limit = self.angular_speeds[self.speed_index]

        # 上次换挡的时间
        self.last_swith_gear_time = time.time()


        
    def buttonCallback(self,joy_data):
        if not isinstance(joy_data, Joy): return
        # print(f"joy_data.axes:{joy_data.axes}")
        self.user_jetson(joy_data)
        
    def user_jetson(self, joy_data:Joy):

        #lock or unlock joy control
        if joy_data.buttons[9] == 1: self.nav_lock_toggle()
        
        #linear Gear control

        if joy_data.buttons[8] == 1:
            curtime = time.time()
            if curtime - self.last_swith_gear_time > 0.3:
                self.speed_index = (self.speed_index + 1) % len(self.x_speeds)
                self.xspeed_limit = self.x_speeds[self.speed_index]
                self.angular_speed_limit = self.angular_speeds[self.speed_index]
                self.last_swith_gear_time = curtime
                self.get_logger().info(f"cur_gear is: {self.speed_index}  {self.xspeed_limit}  {self.angular_speed_limit}")
                

        # joy_data.axes[1]是接收左边操纵杆前后拨动的信号
        xlinear_speed = self.filter_data(joy_data.axes[1]) * self.xspeed_limit

        # joy_data.axes[0]是接收左边操纵杆左右拨动的信号
        angular_speed = self.filter_data(joy_data.axes[0]) * self.angular_speed_limit

        twist = Twist()
        twist.linear.x = xlinear_speed
        twist.angular.z = angular_speed
        if self.Joy_active == True:
            for i in range(3): self.pub_cmdVel.publish(twist)
    
    def filter_data(self, value):
        if abs(value) < 0.2: value = 0
        return value
    
    def nav_lock_toggle(self):
        now_time = time.time()
        if now_time - self.cancel_time > 1:
            Joy_ctrl = Bool()
            self.Joy_active = not self.Joy_active
            if self.Joy_active == True:
                self.get_logger().info("joy control enabled!")
                # print("joy control enabled!")
            else:
                self.get_logger().info("joy control disabled!")
                # print("joy control disabled!")
            Joy_ctrl.data = self.Joy_active
            for i in range(3):
                self.pub_JoyState.publish(Joy_ctrl)
                self.pub_cmdVel.publish(Twist())
            self.cancel_time = now_time
            
def main():
    rclpy.init()
    joy_ctrl = JoyTeleop('joy_ctrl')
    rclpy.spin(joy_ctrl)        

if __name__ == "__main__":
    main()