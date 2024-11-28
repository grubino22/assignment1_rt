#!/usr/bin/env python
import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math
from std_msgs.msg import Float32, Bool

treshold = 1.5;
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
	
	turtle1_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
	turtle2_pub = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)
	
	rospy.Subscriber('/turtle1/pose', Pose, turtle1Callback)
	rospy.Subscriber('/turtle2/pose', Pose, turtle2Callback)
	
	distance_pub = rospy.Publisher('/turtles_distance', Float32, queue_size=10)
	
	movement_flag_pub = rospy.Publisher('/movement_allowed', Bool, queue_size=10)
	
	rospy.init_node('distance_control', anonymous=True)
	rate = rospy.Rate(100)	
	
	while not rospy.is_shutdown():
		
		distance = compute_distance()
		distance_pub.publish(distance)
		
		if distance >= treshold :
			movement_allowed = True;
		
		if distance < treshold:
			rospy.loginfo("The turtles are too close!")
			turtle1_pub.publish(Twist())
			turtle2_pub.publish(Twist())
			movement_allowed = False
			
		if (turtle1_pose.x <= low_boundary or turtle1_pose.y <= low_boundary or turtle1_pose.x >= high_boundary or turtle1_pose.y >= high_boundary):
			rospy.loginfo("turtle1 is too close to the boundary, it's being stopped")
			turtle1_pub.publish(Twist())
			movement_allowed = False
			
		if (turtle2_pose.x <= low_boundary or turtle2_pose.y <= low_boundary or turtle2_pose.x >= high_boundary or turtle2_pose.y >= high_boundary):
			rospy.loginfo("turtle2 is too close to the boundary, it's being stopped")
			turtle2_pub.publish(Twist())
			movement_allowed = False
		
		movement_flag_pub.publish(movement_allowed)
			
		rate.sleep()
	

if __name__ == '__main__':
    try:
        control_distance()
    except rospy.ROSInterruptException:
        pass	

