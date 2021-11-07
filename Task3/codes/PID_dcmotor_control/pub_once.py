#!/usr/bin/env python3
'''
Author : Youssef Abbas 
Date   :  6/11/2021
'''
import rospy
from std_msgs.msg import Float64

def publish_velocity():
    rospy.init_node('set_point')                                   #initiate node to publish set point to Motor
    pub  = rospy.Publisher('drive_motor', Float64)                 #Publish on Topic 'drive motor' data type 'Float64'
    pub.publish(target)


if __name__ == '__main__':
    target  = float(input('Enter Set Point : '))
    publish_velocity()