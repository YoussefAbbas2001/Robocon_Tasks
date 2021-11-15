#!usr/bin/env python 
import rospy
from  std_msgs.msg import Float64

measure = []
def callback(data):
  measure.append(data.data)
  x = KF_1D(measure, r = 1, q = 2)
  pub.publish(x)

	 


def filter_yaw():
  rospy.Subscriber("yaw_topic", Float64,callback)
  rospy.spin()



def KF_1D( Z,r, q):
	# q: process noise variance (uncertainty in the system's dynamic model)
	# r: measurement uncertainty
	# Z: list of position estimates derived from sensor measurements

	# Initialize state (x) and state uncertainty (p)
	x = 0
	p = 100

	for i in range(len(Z)):
		x = x               #previous predict
		p = p + q           #uncertainty
		z = Z[i]            #get Measure
		k = p / ( p + r)    
		x = x + k * (z - x) #update state
		p = (1 - k) * p     #update estimate uncertanity
	return x 

if __name__ ==  "__main__":
  rospy.init_node('filter_node', anonymous=True)
  pub = rospy.Publisher('filtered_yaw', Float64, queue_size=10)
  filter_yaw()
