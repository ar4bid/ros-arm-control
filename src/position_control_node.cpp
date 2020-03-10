#include "ros/ros.h"
#include "cui_encoders/arm_pos_enc_vals.h"
#include "qset_msgs/ArmControl.h"
#include "qset_msgs/TargetAngle.h"
#include "qset_msgs/sabertooth.h"
#include <sensor_msgs/JointState.h>
#include <string.h>
#include "Arm.h"
#include "position_control.h"
#include "angle_control.h"

#define StopCommand 64
#define PI 3.14159265359

using namespace std;

int main(int argc, char *argv[]) 
{
	Arm arm;
	ros::init(argc, argv, "arm_position_control_node");
	ros::NodeHandle n;
	ros::Publisher pub;
	ros::Subscriber joystickSub;
	ros::Subscriber angleSub;
	ros::Subscriber angleActiveSub;

	angleActiveSub = n.subscribe("is_angle_control_active", 1, &Arm::angleActiveCallback, &arm);
	joystickSub = n.subscribe("arm_control", 1, &Arm::joystickCallback, &arm);
	angleSub = n.subscribe("/encoders/arm_control/joint_states", 1, &Arm::angleCallback, &arm);
	pub = n.advertise<qset_msgs::sabertooth>("sabertooth", 1);
	ros::Rate loop_rate(20);
	ros::Rate send_rate(40);
	ros::spinOnce();
	sabertooth::sabertooth msg;
	int lastMode = arm.controlMode;
	float holdX;
	float holdY;
	int motorSpeed;
	ROS_INFO("Position Control Node Ready.");
	while(ros::ok())
	{
		ROS_INFO("Control mode is %d.",arm.controlMode);
		while(arm.controlMode == -1){
			ros::spinOnce();//another node has deactivated this node so wait	
			loop_rate.sleep();
		}	
		if(arm.controlMode != lastMode){
			ROS_INFO("Mode Changed!");
			arm.stopAllJoints(msg, pub, arm);
			updateCurrentPos(arm);
			holdX = arm.x;
			holdY = arm.y;	
			arm.wristHoldAngle = getWristAngle(arm);
		} 
		if(arm.controlMode == qset_msgs::ArmControl::YCONTROL || arm.controlMode == qset_msgs::ArmControl::XCONTROL){
			ROS_INFO("Position Control Node Active.");
			updateCurrentPos(arm);
			if (arm.controlMode == qset_msgs::ArmControl::YCONTROL){
				arm.y += arm.joystickSP;
				arm.x = holdX;
			}
			else if(arm.controlMode == qset_msgs::ArmControl::XCONTROL){
				arm.x += arm.joystickSP;
				arm.y = holdY;
			}
			updateTargetAngles(arm);
			ROS_INFO("pan SP is: %f", arm.angleSP[0]);
			ROS_INFO("shoulder SP is: %f", arm.angleSP[1]);
			ROS_INFO("elbow SP is: %f", arm.angleSP[2]);
			ROS_INFO("wrist SP is: %f", arm.angleSP[3]);
			ROS_INFO("arm X: %f", arm.x);
			ROS_INFO("arm Y: %f", arm.y);
			for(int i=1;i<4;i++){			
				if(i == 3)//Doing wrist stuff
				{
					motorSpeed = angleControl(i, arm);
					msg.data = motorSpeed;
					msg.address = arm.motorAddress[3][0];
					msg.command = arm.motorAddress[3][1];
					pub.publish(msg);
					send_rate.sleep();
					msg.data = ((motorSpeed-64)*-1)+64;
					msg.address = arm.motorAddress[4][0];
					msg.command = arm.motorAddress[4][1];
					pub.publish(msg);
					send_rate.sleep();
				}
				else
				{
					msg.address = arm.motorAddress[i][0];
					msg.command = arm.motorAddress[i][1];
					msg.data = angleControl(i, arm);
					ROS_INFO("Publishing to joint %d with value %d.",i, msg.data);
					pub.publish(msg);
					send_rate.sleep();
				}
			}
		}
		lastMode = arm.controlMode;
		ros::spinOnce();
		loop_rate.sleep();
	}//end while(ros::ok())
	return 0;
}
