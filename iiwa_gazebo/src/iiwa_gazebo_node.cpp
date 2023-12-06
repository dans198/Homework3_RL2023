#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>


int main(int argc, char* argv[])
{ 	
	ros::init(argc, argv, "controller");
	ros::NodeHandle nodeHandle;

	ros::Rate loopRate(10);
	
	
	ros::Publisher publisher1 = nodeHandle.advertise<std_msgs::Float64>("/iiwa/PositionJointInterface_J1_controller/command", 1);
	ros::Publisher publisher2 = nodeHandle.advertise<std_msgs::Float64>("/iiwa/PositionJointInterface_J2_controller/command", 1);
	ros::Publisher publisher3 = nodeHandle.advertise<std_msgs::Float64>("/iiwa/PositionJointInterface_J3_controller/command", 1);
	ros::Publisher publisher4 = nodeHandle.advertise<std_msgs::Float64>("/iiwa/PositionJointInterface_J4_controller/command", 1);
	ros::Publisher publisher5 = nodeHandle.advertise<std_msgs::Float64>("/iiwa/PositionJointInterface_J5_controller/command", 1);
	ros::Publisher publisher6 = nodeHandle.advertise<std_msgs::Float64>("/iiwa/PositionJointInterface_J6_controller/command", 1);
	ros::Publisher publisher7 = nodeHandle.advertise<std_msgs::Float64>("/iiwa/PositionJointInterface_J7_controller/command", 1);


	
	while (ros::ok()) {
	std_msgs::Float64 msg1;
	std_msgs::Float64 msg2;
	std_msgs::Float64 msg3;
	std_msgs::Float64 msg4;
	std_msgs::Float64 msg5;
	std_msgs::Float64 msg6;
	std_msgs::Float64 msg7;

	
	msg1.data=0.0;
	msg2.data=1.57;
	msg3.data=-1.57;
	msg4.data=-1.57;
	msg5.data=1.57;
	msg6.data=-1.57;
	msg7.data=1.57;

	//ROS_INFO_STREAM(message.data);
	
	publisher1.publish(msg1);
	publisher2.publish(msg2);
	publisher3.publish(msg3);
	publisher4.publish(msg4);
	publisher5.publish(msg5);
	publisher6.publish(msg6);
	publisher7.publish(msg7);
	
	ros::spinOnce();
	loopRate.sleep();
	
	} 
	

	return 0;
}