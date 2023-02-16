import os
from typing import Tuple
import numpy as np
import rospy
import ctypes
import smbus2
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String, ColorRGBA
from smbus2 import SMBus
from duckietown_msgs.msg import WheelsCmdStamped, WheelEncoderStamped
from sensor_msgs.msg import Range
import time
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
        self.temp_d_straight = 0
    def callback(self, data):
        self.range = data.range
        #rospy.loginfo("Kuulen: %s", data.range)
    def rightwheel(self, data):
        self.right = data.data
        #rospy.loginfo("Parem ratas: %s", data.data)        #nende funktsioonidega saame andmeid subscribidest kätte
    def leftwheel(self, data):
        self.left = data.data
        #rospy.loginfo("Vasak ratas: %s", data.data)
    def time_leftwheel(self, data):
        self.timeL = data.header.seq
    def time_rightwheel(self, data):
        self.timeR = data.header.seq
   
    def on_shutdown(self):
        speed.vel_left = 0
        speed.vel_right = 0
        self.pub.publish(speed)
        #bus.close()
        rospy.on_shutdown()
    def run(self):
        bus = SMBus(15) #1 hz
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            bus = SMBus(15)
            read = bus.read_byte_data(62,17)
            try:
                temp = bus.read_byte_data(62, 17)
            except:
                print("ei saa andmeid lugeda")
            message = str(temp)
            rospy.loginfo("Line follower: %s" % message)
            N_tot = 135 # total number of ticks per revolution
            alpha = 2 * np.pi / N_tot # wheel rotation per tick in radians. The angular resolution of our encoders is: {np.rad2deg(alpha)} degrees)
            self.ticks_right = self.right
            self.ticks_left = self.left
            self.delta_ticks_left = self.ticks_left-self.prev_tick_left # delta ticks of left wheel
            self.delta_ticks_right = self.ticks_right-self.prev_tick_right # delta ticks of right wheel
            self.rotation_wheel_left = alpha * self.delta_ticks_left # total rotation of left wheel
            self.rotation_wheel_right = alpha * self.delta_ticks_right # total rotation of right wheel
            print(f"The left wheel rotated: ", np.rad2deg(self.rotation_wheel_left), "degrees")
            print(f"The right wheel rotated: ", np.rad2deg(self.rotation_wheel_right), "degrees")
            R = 0.067           # insert value measured by ruler, in *meters* ratta tolli mõõt pmst
            d_left = R * self.rotation_wheel_left
            d_right = R * self.rotation_wheel_right
            self.d_straight = (d_left+d_right)/2*100
            print(f"The left wheel travelled: {d_left} meters")
            print(f"The right wheel travelled: {d_right} meters")
            # How much has the robot rotated?
            self.kaugus_cm = round(self.range*100, 1)
            print("kaugus on: ",self.kaugus_cm, "Cm")
            Delta_Theta = (d_right-d_left)/self.baseline_wheel2wheel # expressed in radians
            print(f"The robot has rotated: {np.rad2deg(Delta_Theta)} degrees")
            self.prev_tick_left = self.ticks_left
            self.prev_tick_right = self.ticks_right
            print("Left wheel time: ", self.timeL)
            print("Right wheel time: ", self.timeR)
            #theta_ref = ctypes.c_int8(temp)
            #Jaab seisma kui sein:

            def kastistmooda():
                while self.kaugus_cm <= 5.0: # and self.phooramine <= 90:
                    speed.vel_right = 1
                    speed.vel_left = 0
                    self.pub.publish(speed)
                    self.kaugus_cm = round(self.range*100, 1)
                time.sleep(0.2)
                    #self.phooramine = self.phooramine + float(np.rad2deg(Delta_Theta))
                while self.temp_d_straight < 20:
                    speed.vel_right=0.2
                    speed.vel_left=0.2
                    self.pub.publish(speed)
                    self.temp_d_straight = self.temp_d_straight + self.d_straight
                time.sleep(0.2)
                speed.vel_right= 0
                speed.vel_left = 1
                rate.sleep()
            kastistmooda()
            #-------------P controller---------------------------------------
            print("Lugeja väärtus 10 süsteemis: "+str(read))# algne 10 süsteemi väärtus
            print("2 süsteemi väärtus: "+str(bin(read)[2:].zfill(8)))# 10 tehtud 2 süsteemi
            kahendsusteem = bin(read)[2:].zfill(8)
            masiiv = [-16,-12,-8,-4,4,8,12,16]
            print(kahendsusteem)
            väärtus = []
            for indx, ele in enumerate(kahendsusteem):
                if ele == "1":
                    väärtus.append(masiiv[indx])
            print(väärtus)
            sum = 0
            for value in väärtus:
                sum =sum + value
            print(sum)
            if len(väärtus) != 0:
                err = (sum / len(väärtus)) * 0.05
            print(len(väärtus))
            #------------------------------------------------------------------------------------ eelmise ratta kiirus salvestada
            speed.vel_left = 0.3 + err
            speed.vel_right = 0.3 - err
            self.pub.publish(speed)
            rate.sleep()
            bus.close()
            #print(speed.vel_left)
        # print(speed.vel_right )
if __name__ == '__main__':
    # create the node
    node = MyPublisherNode(node_name='my_publisher_node')
    # run node
    node.run()
    # keep spinning
    rospy.spin()