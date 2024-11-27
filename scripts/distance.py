#!/usr/bin/env python
import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math

def turtle1Callback(msg):
	global turtle1_pose
	turtle1_pose = msg

def turtle2Callback(msg):
	global turtle2_pose
	turtle2_pose = msg
	
def compute_distance():
	return math.sqrt((turtle1_pose.x-turtle2_pose.x)**2+(turtle1_pose.y-turtle2_pose.y)**2)

def control_distance():
	
	turtle1_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
	turtle1_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
	
	rospy.Subscriber('/turtle1/pose', Pose, turtle1Callback)
	rospy.Subscriber('/turtle2/pose', Pose, turtle2Callback)
	
	rospy.init_node('distance_control', anonymous=True)
	rate = rospy.Rate(100)	
	
	while not rospy.is_shutdown():
		
		distance = compute_distance()
		print("Distance: ", distance)
		
			
		rate.sleep()
	

if __name__ == '__main__':
    try:
        control_distance()
    except rospy.ROSInterruptException:
        pass	

