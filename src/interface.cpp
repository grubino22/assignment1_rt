#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "turtlesim/Spawn.h"
#include "geometry_msgs/Twist.h"
#include <string>

void Callback(const turtlesim::Pose::ConstPtr& msg) {

}

int main (int argc, char **argv) {
	
	ros::init(argc, argv, "UI");
	
	ros::NodeHandle n;
	
	ros::Publisher turtle1_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 100);
	ros::Publisher turtle2_pub = n.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 100);
	
	ros::Subscriber turtle1_sub = n.subscribe("/turtle1/pose", 100, Callback);
	ros::Subscriber turtle2_sub = n.subscribe("/turtle2/pose", 100, Callback);
	
	ros::ServiceClient client1 = n.serviceClient<turtlesim::Spawn>("/spawn");
	turtlesim::Spawn spawn1;
	spawn1.request.name = "turtle2";
	spawn1.request.x = 2.0;
	spawn1.request.y = 2.0;
	spawn1.request.theta = 0.0;
	client1.call(spawn1);
	
	std::string control_name;
	double lin_vel;
	double ang_vel;
	
	while(ros::ok()) {
		
		std::cout << "Choose the turtle to control (turtle1 or turtle2): ";
		std::cin >> control_name;
		if(control_name!="turtle1" && control_name!="turtle2"){
			std::cout << "Turtle name not valid. Enter 'turtle1' or 'turtle2'." << std::endl;
			continue;
		}
		
		std::cout << "Choose the linear velocity: ";
		std::cin >> lin_vel;
		
		std::cout << "Choose the angular velocity: ";
		std::cin >> ang_vel;
		
		geometry_msgs::Twist input_vel;
		input_vel.linear.x = lin_vel;
		input_vel.angular.z = ang_vel;
		
		ros::Rate loop_rate(100);
		
		// Move the turtle for 1 second
		for(int i=0; i<100; ++i) {
			if (control_name == "turtle1") {turtle1_pub.publish(input_vel);}
			else if (control_name == "turtle2") {turtle2_pub.publish(input_vel);}
			
			ros::spinOnce();
			loop_rate.sleep();
		}
		
		// Stop the turtle
		input_vel.linear.x = 0.0;
		input_vel.angular.z = 0.0;
		if (control_name == "turtle1") {turtle1_pub.publish(input_vel);}
		else if (control_name == "turtle2") {turtle2_pub.publish(input_vel);}
		
		
		ros::spinOnce();
		loop_rate.sleep();
	
	}
	
	
	
	return 0;
}
