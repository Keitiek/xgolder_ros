#!/usr/bin/env python3
import os
import rospy
import smbus2
import time
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
from smbus2 import SMBus
from duckietown_msgs.msg import WheelsCmdStamped
from sensor_msgs.msg import Range


speed = WheelsCmdStamped()

class MyPublisherNode(DTROS):
    
    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MyPublisherNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # construct publisher
        self.pub = rospy.Publisher('/xgolder/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=10)
        self.tof = rospy.Subscriber('/xgolder/front_center_tof_driver_node/range', Range, self.callback)

        self.range = 0
        self.sein = 0
    
    def callback(self, data):
        self.range = data.range

    def on_shutdown(self):
        speed.vel_left = 0
        speed.vel_right = 0
        self.pub.publish(speed)
        rospy.on_shutdown()
        
    def run(self):
        # publish message every 1 second
        rate = rospy.Rate(20) # 1Hz
        
        integral= 0 
        start_time = time.time()
        prev_error = 0
        
        # Set initial parameters for duck's devel run ( Kp, Ki, Kd, v_max).
        rospy.set_param("/pidv", [0.03, 0.0069, 0.05, 0.3005])
        
        while not rospy.is_shutdown(): 
            #Tof range in cm
            self.kaugus_cm = round(self.range*100, 1)
            print("kaugus on: ",self.kaugus_cm, "Cm")    
       
            #Tof range in cm
            
            bus = SMBus(15)
            read = bus.read_byte_data(62,17)
            
            dt = time.time()-start_time
            
             # Get parameters from ROS
            Kp, Ki, Kd, v_max = rospy.get_param("/pidv")

            #----Proportional controller-----
            #print("Lugeja emty_array 10 süsteemis: "+str(read))# algne 10 süsteemi emty_array
            #print("2 süsteemi emty_array: "+str(bin(read)[2:].zfill(8)))# 10 tehtud 2 süsteemi
            
            binary = bin(read)[2:].zfill(8)
            array = [-16,-12,-8,-4,4,8,12,16]
            
            #print(binary)
            
            emty_array = []
            
            for indx, ele in enumerate(binary):
                if ele == "1":
                    emty_array.append(array[indx])   
            print(emty_array)
           
            sum = 0
            for value in emty_array:
                sum =sum + value      

            print(sum)
            
            if len(emty_array) != 0:
                err = (sum / len(emty_array)) 
            print(len(emty_array))
            
            P = Kp * err
           
           #-------Integral controller-------
        
            
            integral = integral + err *dt
            
            I = Ki*(integral)
            
            
            #-------Derivative controller------

            derivative = err - prev_error
            
            D = Kd * derivative/dt

             #---------- sõitmine
            PID = min(max(P + I + D,-0.7), 0.7)

            speed.vel_left =  max(0,v_max + PID)
            speed.vel_right = max(0,v_max - PID)
              
                
            self.pub.publish(speed)
            rate.sleep()
            bus.close()
            
            #print(speed.vel_left)
           # print(speed.vel_right )

            
            prev_error = err
            start_time = time.time()
        
            if self.kaugus_cm <= 20.0: 
                speed.vel_right = 0.1
                speed.vel_left = 0
                self.pub.publish(speed)
                self.kaugus_cm = round(self.range*100, 1)
                time.sleep(0.6)
                speed.vel_right = 0.1
                speed.vel_left = 0.1
                self.pub.publish(speed)
                time.sleep(2.2)
                speed.vel_right = 0
                speed.vel_left = 0.1
                self.pub.publish(speed)
                time.sleep(0.6)
            else:             
                 rate.sleep()
                        

if __name__ == '__main__':
    # create the node
    node = MyPublisherNode(node_name='my_publisher_node')
    # run nodevõiks olla mõned funktsioonid määratud
    node.run()
    # keep spinning
    rospy.spin()
    