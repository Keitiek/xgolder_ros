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
            
            bus = SMBus(15)
            read = bus.read_byte_data(62,17)

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
    