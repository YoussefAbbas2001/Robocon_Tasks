#!/usr/bin/env python3
'''
Author : Youssef Abbas 
Date   :  6/11/2021
'''
import rospy
from std_msgs.msg import Float64

def publish_velocity():
    rospy.init_node('set_point')                                   #initiate node to publish set point to Motor
    pub  = rospy.Publisher('drive_motor', Float64,queue_size=1)  #Publish on Topic 'drive motor' data type 'Float64'
    rate = rospy.Rate(1)                                           #Rate of Node. Make it 1 HZ
    

    while not rospy.is_shutdown():
        pub.publish(target)
        rate.sleep()


if __name__ == '__main__':
    target  = float(input('Enter Set Point : '))
    publish_velocity()