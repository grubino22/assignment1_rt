#!/usr/bin/env python
import rospy
from turtlesim.msg import Pose
import math
from std_msgs.msg import Float32, Bool

treshold = 1.5
high_boundary = 10
low_boundary = 1

turtle1_pose = Pose()
turtle2_pose = Pose()
turtle1_ready = False
turtle2_ready = False
first = True;

def turtle1Callback(msg):
	global turtle1_pose, turtle1_ready
	turtle1_pose = msg
	turtle1_ready = True

def turtle2Callback(msg):
	global turtle2_pose, turtle2_ready
	turtle2_pose = msg
	turtle2_ready = True
	
def compute_distance():
	return math.sqrt((turtle1_pose.x-turtle2_pose.x)**2+(turtle1_pose.y-turtle2_pose.y)**2)

def control_distance():
	
	rospy.init_node('distance_control', anonymous=True)
	
	rospy.Subscriber('/turtle1/pose', Pose, turtle1Callback)
	rospy.Subscriber('/turtle2/pose', Pose, turtle2Callback)
	
	distance_pub = rospy.Publisher('/turtles_distance', Float32, queue_size=10)
	
	movement_flag_pub = rospy.Publisher('/movement_allowed', Bool, queue_size=10)
	
	rate = rospy.Rate(100)	
	
	while not rospy.is_shutdown():
		
		global first
		if first:
			rospy.sleep(0.5)
			first = False
			
		if not turtle1_ready or not turtle2_ready:
			rospy.loginfo("One or both turtles are not spawned!")
			rate.sleep()
			continue
		
		distance = compute_distance()
		distance_pub.publish(distance)
		
		if distance < treshold:
			rospy.loginfo("The turtles are too close!")
			movement_allowed = False
		elif (turtle1_pose.x <= low_boundary or turtle1_pose.y <= low_boundary or turtle1_pose.x >= high_boundary or turtle1_pose.y >= high_boundary):
			rospy.loginfo("turtle1 is too close to the boundary, it's being stopped")
			movement_allowed = False
		elif (turtle2_pose.x <= low_boundary or turtle2_pose.y <= low_boundary or turtle2_pose.x >= high_boundary or turtle2_pose.y >= high_boundary):
			rospy.loginfo("turtle2 is too close to the boundary, it's being stopped")
			movement_allowed = False
		else:
			movement_allowed = True;
		
		movement_flag_pub.publish(movement_allowed)
			
		rate.sleep()
	

if __name__ == '__main__':
    try:
        control_distance()
    except rospy.ROSInterruptException:
        pass	

