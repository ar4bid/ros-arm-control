#include "ros/ros.h"
#include "cui_encoders/arm_pos_enc_vals.h"
#include "qset_msgs/ArmControl.h"
#include "qset_msgs/sabertooth.h"
#include "qset_msgs/TargetAngle.h"
#include <sensor_msgs/JointState.h>
#include <string.h>
#include <ros/ros.h>
#include <ros/ros.h>
#include "angle_control.h"
#include "Arm.h"

int main(int argc, char *argv[])
{
	Arm arm;
	ros::init(argc, argv, "arm_position_control_node");
	ros::NodeHandle n;
	ros::Publisher pub;
	ros::Publisher activePub;
	ros::Subscriber armSub;
	ros::Subscriber jointSub;
	ros::Subscriber angleSub;
	jointSub = n.subscribe("/encoders/arm_control/joint_states", 1, &Arm::angleCallback, &arm);
	angleSub = n.subscribe("angle_commands", 1, &Arm::angleSPCallback, &arm);
	pub = n.advertise<qset_msgs::sabertooth>("sabertooth", 1);
	activePub = n.advertise<std_msgs::Bool>("is_angle_control_active", 1);
	ros::Rate send_rate(40);	
	ros::Rate loop_rate(20);
	arm.angleSP[0] = -1;
	arm.angleSP[1] = -1;
	arm.angleSP[2] = -1;
	arm.angleSP[3] = -1;
	std_msgs::Bool deactivateNodes;
    qset_msgs::sabertooth msg;
	int motorSpeed;
	while(ros::ok())
	{
		ROS_INFO("angleSPs are pan: %f, shoulder: %f, elbow: %f, wrist: %f", arm.angleSP[0], arm.angleSP[1], arm.angleSP[2], arm.angleSP[3]);
		for(int i = 0; i<4; i++)//Loops through PAN, SHOULDER, ELBOW
		{
			if(arm.angleSP[i] != -1){
				deactivateNodes.data = true; //deactivate the other control nodes
				activePub.publish(deactivateNodes);
				msg.address = arm.motorAddress[i][0];
				msg.command = arm.motorAddress[i][1];
				motorSpeed = angleControl(i, arm);
				msg.data = motorSpeed; 
				pub.publish(msg);
				send_rate.sleep();
				if(i == int(arm_control::ArmControl::WRIST_TILT)){ //if using wrist send msg to other motor too
					msg.address = arm.motorAddress[i+1][0];
					msg.command = arm.motorAddress[i+1][1];
					msg.data = ((motorSpeed-64)*-1)+64;
					pub.publish(msg);
					send_rate.sleep(); 
				}
				if(motorSpeed <= 67 && motorSpeed >= 61)//the motor won't produce enough torque to move in this range
					arm.angleSP[i] = -1;//set to -1 when all joints have arrived so a different control mode can take over again
			}
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
