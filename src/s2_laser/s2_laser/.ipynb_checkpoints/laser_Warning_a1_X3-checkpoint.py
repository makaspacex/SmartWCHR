#ros lib
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
#commom lib
import math
import numpy as np
import time
from time import sleep
from s2_laser.common import *
print ("improt done")
RAD2DEG = 180 / math.pi

class laserWarning(Node):
    def __init__(self,name):
        super().__init__(name)
        #create a sub
        self.sub_laser = self.create_subscription(LaserScan,"/scan",self.registerScan,1)
        self.sub_JoyState = self.create_subscription(Bool,'/JoyState', self.JoyStateCallback,1)
        #create a pub
        self.pub_vel = self.create_publisher(Twist,'/cmd_vel',1)
        self.pub_Buzzer = self.create_publisher(Bool,'/Buzzer',1)
        
        
        
        
        #declareparam
        self.declare_parameter("linear",0.5)
        self.linear = self.get_parameter('linear').get_parameter_value().double_value
        self.declare_parameter("angular",1.0)
        self.angular = self.get_parameter('angular').get_parameter_value().double_value
        self.declare_parameter("LaserAngle",40.0)
        self.LaserAngle = self.get_parameter('LaserAngle').get_parameter_value().double_value
        self.declare_parameter("ResponseDist",0.55)
        self.ResponseDist = self.get_parameter('ResponseDist').get_parameter_value().double_value
        self.declare_parameter("Switch",False)
        self.Switch = self.get_parameter('Switch').get_parameter_value().bool_value
        
        self.Right_warning = 0
        self.Left_warning = 0
        self.front_warning = 0
        self.Joy_active = False
        self.ros_ctrl = SinglePID()
        self.ang_pid = SinglePID(3.0, 0.0, 5.0)
        self.Buzzer_state = False
        self.Moving = False
        
        self.timer = self.create_timer(0.01,self.on_timer)
        
    def on_timer(self):
        self.Switch = self.get_parameter('Switch').get_parameter_value().bool_value
        self.angular = self.get_parameter('angular').get_parameter_value().double_value
        self.linear = self.get_parameter('linear').get_parameter_value().double_value
        self.LaserAngle = self.get_parameter('LaserAngle').get_parameter_value().double_value
        self.ResponseDist = self.get_parameter('ResponseDist').get_parameter_value().double_value

    def JoyStateCallback(self, msg):
        if not isinstance(msg, Bool): return
        self.Joy_active = msg.data

    def registerScan(self, scan_data):
        if not isinstance(scan_data, LaserScan): return
        ranges = np.array(scan_data.ranges)
        minDistList = []
        minDistIDList = []
        
        for i in range(len(ranges)):
            angle = (scan_data.angle_min + scan_data.angle_increment * i) * RAD2DEG
            if abs(angle) > (180 - self.LaserAngle):
            	minDistList.append(ranges[i])
            	minDistIDList.append(angle)
        if len(minDistList) == 0: return
        minDist = min(minDistList)
        minDistID = minDistIDList[minDistList.index(minDist)]
        if self.Joy_active or self.Switch == True:
        	if self.Moving == True:
        		self.pub_vel.publish(Twist())
        		self.Moving = not self.Moving
        	return
        self.Moving = True
        if minDist <= self.ResponseDist:
        	if self.Buzzer_state == False:
        		b = Bool()
        		b.data = True
        		self.pub_Buzzer.publish(b)
        		self.Buzzer_state = True
        else:
        	if self.Buzzer_state == True:
        		self.pub_Buzzer.publish(Bool())
        		self.Buzzer_state = False
        velocity = Twist()
        ang_pid_compute = self.ang_pid.pid_compute((180 - abs(minDistID)) / 36, 0)
        if minDistID > 0: velocity.angular.z = -ang_pid_compute
        else: velocity.angular.z = ang_pid_compute
        if ang_pid_compute < 0.02: velocity.angular.z = 0.0
        self.pub_vel.publish(velocity)
        

def main():
    rclpy.init()
    laser_warn = laserWarning("laser_Warnning_a1")
    print ("start it")
    try:
        rclpy.spin(laser_warn)
    except KeyboardInterrupt:
        pass
    finally:
        laser_warn.pub_vel.publish(Twist())
        laser_warn.pub_Buzzer.publish(Bool())
        laser_warn.destroy_node()
        rclpy.shutdown()
