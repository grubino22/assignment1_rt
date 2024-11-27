#!/usr/bin/env python
import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

def Callback(data):
	

def control_distance():
	
	turtle1_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
	turtle1_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
	
	rospy.init_node('distance_control', anonymous=True)
	rate = rospy.Rate(100)	
	
	while not rospy.is_shutdown():
		
		
		
			
		rate.sleep()
	

if __name__ == '__main__':
    try:
        control_distance()
    except rospy.ROSInterruptException:
        pass	

