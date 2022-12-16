#!/usr/bin/env python3
import os
import rospy
import smbus2
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
from smbus2 import SMBus
from duckietown_msgs.msg import WheelsCmdStamped

speed = WheelsCmdStamped()

class MyPublisherNode(DTROS):
    
    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MyPublisherNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # construct publisher
        self.pub = rospy.Publisher('/xgolder/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=10)
        
    def on_shutdown(self):
        speed.vel_left = 0
        speed.vel_right = 0
        self.pub.publish(speed)
        rospy.on_shutdown()
        
    def run(self):
        # publish message every 1 second
        rate = rospy.Rate(20) # 1Hz
        while not rospy.is_shutdown():
            
            bus = SMBus(1)
            read = bus.read_byte_data(62,17)

            if read == 1:
                speed.vel_left = -0.25
                speed.vel_right = 0.25
            elif read == 3:
                speed.vel_left = 0.25
                speed.vel_right = 0.25
            elif read == 2:
                speed.vel_left = 0.25
                speed.vel_right = 0.25
            elif read == 6:
                speed.vel_left = 0.25
                speed.vel_right = 0.25
            elif read == 4:
                speed.vel_left = 0.25
                speed.vel_right = 0.25
            elif read == 12:
                speed.vel_left = 0.25
                speed.vel_right = 0.25
            elif read == 8:
                speed.vel_left = 0.25
                speed.vel_right = 0.25
            elif read == 24:
                speed.vel_left = 0.25
                speed.vel_right = 0.25
            elif read == 16:
                speed.vel_left = 0.25
                speed.vel_right = 0.25
            elif read == 48:
                speed.vel_left = 0.25
                speed.vel_right = 0.25
            elif read == 32:
                speed.vel_left = 0.25
                speed.vel_right = 0.25
            elif read == 96:
                speed.vel_left = 0.25
                speed.vel_right = 0.25
            elif read == 64:
                speed.vel_left = 0.25
                speed.vel_right = 0.25
            elif read == 192:
                speed.vel_left = 0.25
                speed.vel_right = 0.25
            elif read == 128:
                speed.vel_left = 0.25
                speed.vel_right = 0.25
            self.pub.publish(speed)
            rate.sleep()
            bus.close()
            print(read)
            
if __name__ == '__main__':
    # create the node
    node = MyPublisherNode(node_name='my_publisher_node')
    # run node
    node.run()
    # keep spinning
    rospy.spin()











