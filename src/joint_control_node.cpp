#include "ros/ros.h"
#include "qset_msgs/ArmControl.h"
#include "qset_msgs/sabertooth.h"
#include <sensor_msgs/JointState.h>
#include <string.h>
#include <ros/ros.h>
#include <ros/ros.h>
#include "std_msgs/Int16MultiArray.h"
#include "Arm.h"
#include "joint_control.h"

int main(int argc, char *argv[])
{
	Arm arm;
	ros::init(argc, argv, "arm_voltage_control_node");
	ros::NodeHandle n;
	ros::Publisher pub;
	ros::Subscriber joystickSub;
	ros::Subscriber jointSub;
	ros::Subscriber angleActiveSub;

	angleActiveSub = n.subscribe("is_angle_control_active", 1, &Arm::angleActiveCallback, &arm);
	joystickSub = n.subscribe("arm_control", 1, &Arm::joystickCallback, &arm);
	pub = n.advertise<qset_msgs::sabertooth>("sabertooth", 1);

	ros::Rate loop_rate(20);
	ros::Rate send_rate(20);
	sabertooth::sabertooth msg;	

	int lastMode;
	ROS_INFO("Starting joint control.");
	while(ros::ok())
	{
		while(arm.controlMode == -1){ //wait for the first message to arrive
			ros::spinOnce(); //this node has not been activated, wait until it is
			loop_rate.sleep();
			ROS_INFO("Waiting on joint message");
			ROS_INFO("Control mode is %d.",arm.controlMode);
		}
		if(arm.controlMode != lastMode){
			ROS_INFO("Mode change! Writing stop.");
			arm.stopAllJoints(msg, pub, arm);
		}
		//Wrist roll
		if(arm.controlMode == qset_msgs::ArmControl::WRIST_ROLL){
			arm.wristSpeed[0] = jointControl(arm.joystickSP, arm);
			arm.wristSpeed[1] = jointControl(arm.joystickSP, arm);
			for(int i=0; i<2; i++){
				msg.data = arm.wristSpeed[i];
				msg.address = arm.motorAddress[3+i][0];
				msg.command = arm.motorAddress[3+i][1];
				pub.publish(msg);
				send_rate.sleep();
			}	
		}
		//Wrist Tilt
		else if(arm.controlMode == qset_msgs::ArmControl::WRIST_TILT){
			arm.wristSpeed[0] = jointControl(-arm.joystickSP, arm);
			arm.wristSpeed[1] = jointControl(arm.joystickSP, arm);
			for(int i=0; i<2; i++){
				msg.data = arm.wristSpeed[i];
				msg.address = arm.motorAddress[3+i][0];
				msg.command = arm.motorAddress[3+i][1];
				pub.publish(msg);
				send_rate.sleep();
			}
		}
		//Grab
		else if(arm.controlMode == qset_msgs::ArmControl::GRAB){
			msg.data = jointControl(arm.joystickSP, arm);
			msg.address = arm.motorAddress[5][0];
			msg.command = arm.motorAddress[5][1];
			pub.publish(msg);
			send_rate.sleep();
		}
		//Everything else
		else if(arm.controlMode == qset_msgs::ArmControl::PAN || arm.controlMode == qset_msgs::ArmControl::SHOULDER || arm.controlMode == qset_msgs::ArmControl::ELBOW){
			msg.data = jointControl(arm.joystickSP, arm);	
			msg.address = arm.motorAddress[arm.controlMode][0];
			msg.command = arm.motorAddress[arm.controlMode][1];
			pub.publish(msg);
			send_rate.sleep();
		}
		lastMode = arm.controlMode;
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
} 
