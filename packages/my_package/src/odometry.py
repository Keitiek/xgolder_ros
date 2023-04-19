#!/usr/bin/env python3
import os
from typing import Tuple
import numpy as np
import rospy
import ctypes
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String, ColorRGBA
from smbus2 import SMBus
from duckietown_msgs.msg import WheelsCmdStamped, WheelEncoderStamped
from sensor_msgs.msg import Range

speed = WheelsCmdStamped()

class MyPublisherNode(DTROS):
    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MyPublisherNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # construct publisher
        self.pub = rospy.Publisher('/xgolder/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=10)
        self.tof = rospy.Subscriber('/xgolder/front_center_tof_driver_node/range', Range, self.callback)
        self.rwheel = rospy.Subscriber('/xgolder/right_wheel_encoder_node/tick', WheelEncoderStamped ,self.rightwheel)
        self.lwheel = rospy.Subscriber('/xgolder/left_wheel_encoder_node/tick', WheelEncoderStamped, self.leftwheel)
        self.seqLeft = rospy.Subscriber('/xgolder/left_wheel_encoder_node/tick', WheelEncoderStamped, self.time_leftwheel)
        self.seqRight = rospy.Subscriber('/xgolder/right_wheel_encoder_node/tick', WheelEncoderStamped ,self.time_rightwheel)
    
        self.range = 1
        self.right = 0
        self.left = 0             #alustab nende andmetega, et neid hiljem saaks kutsuda välja 
        self.timeL = 0
        self.timeR = 0
        
        self.ticks_left = 0
        self.prev_tick_left = 0
        self.ticks_right = 0
        self.prev_tick_right = 0
        self.rotation_wheel_left = 0
        self.rotation_wheel_right = 0
        self.delta_ticks_left = 0
        self.delta_ticks_right = 0
        self.baseline_wheel2wheel = 0.095 #  Distance between the center of the two wheels, expressed in meters
        self.x_curr = 0
        self.y_curr = 0
        self.theta_curr = 0
        self.prev_int = 0
        self.prev_e = 0
        self.phooramine = 0
        
#----------- Nende funktsioonidega saame andmeid subscribidest kätte-----------------------------------------------
    def rightwheel(self, data):
        self.right = data.data
        #rospy.loginfo("Parem ratas: %s", data.data)
    
    def leftwheel(self, data):
        self.left = data.data
        #rospy.loginfo("Vasak ratas: %s", data.data)
    
    def time_leftwheel(self, data):
        self.timeL = data.header.seq
    
    def time_rightwheel(self, data):
        self.timeR = data.header.seq


def talker():
    pub = rospy.Publisher('/xgolder/odometry', String, queue_size=10)
    rospy.init_node('odometry', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass