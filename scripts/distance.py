#!/usr/bin/env python
import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math

treshold = 1.0;
high_boundary = 10;
low_boundary = 1;

def turtle1Callback(msg):
	global turtle1_pose
	turtle1_pose = msg

def turtle2Callback(msg):
	global turtle2_pose
	turtle2_pose = msg
	
def compute_distance():
	return math.sqrt((turtle1_pose.x-turtle2_pose.x)**2+(turtle1_pose.y-turtle2_pose.y)**2)

def control_distance():
	
	global turtle1_pub
	turtle1_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
	global turtle2_pub
	turtle2_pub = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)
	
	rospy.Subscriber('/turtle1/pose', Pose, turtle1Callback)
	rospy.Subscriber('/turtle2/pose', Pose, turtle2Callback)
	
	rospy.init_node('distance_control', anonymous=True)
	rate = rospy.Rate(100)	
	
	while not rospy.is_shutdown():
		
		distance = compute_distance()
		if distance < treshold:
			print("The turtles are too close!")
			turtle1_pub.publish(Twist())
			turtle2_pub.publish(Twist())
			
		if (turtle1_pose.x <= low_boundary or turtle1_pose.y <= low_boundary or turtle1_pose.x >= high_boundary or turtle1_pose.y >= high_boundary):
			print("turtle1 is too close to the boundary, it's being stopped")
			turtle1_pub.publish(Twist())
			
		if (turtle2_pose.x <= low_boundary or turtle2_pose.y <= low_boundary or turtle2_pose.x >= high_boundary or turtle2_pose.y >= high_boundary):
			print("turtle2 is too close to the boundary, it's being stopped")
			turtle2_pub.publish(Twist())
		
		
			
		rate.sleep()
	

if __name__ == '__main__':
    try:
        control_distance()
    except rospy.ROSInterruptException:
        pass	

