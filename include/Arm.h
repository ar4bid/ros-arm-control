#ifndef ARM_H
#define ARM_H

#include "ros/ros.h"
#include "cui_encoders/arm_pos_enc_vals.h"
#include <qset_msgs/TargetAngle.h>
#include <qset_msgs/ArmControl.h>
#include <string.h>
#include <std_msgs/Bool.h>
#include "qset_msgs/sabertooth.h"

using namespace std;

/*
Array elements represent differnt joints. 
0 --> Pan
1 --> Shoulder
2 --> Elbow
*/

//TODO define getters and setters and make all other members except the constructor private

class Arm{
	public:
		Arm();
		void initializeAngleSP();
		void joystickCallback(const qset_msgs::ArmControl);
		void angleCallback(const cui_encoders::arm_pos_enc_vals);
		void angleSPCallback(const qset_msgs::TargetAngle);
		void angleActiveCallback(const std_msgs::Bool);
		void paramSetting(const ros::NodeHandle);
		void stopAllJoints(qset_msgs::sabertooth, ros::Publisher, Arm&);
		float x, y, velocity[4], velocitySP[4];
		const float minSpeed = 10, maxSpeed = 117;
		int controlMode = -1;
		float joystickSP;
		int wristSpeed[2];
		float firstLinkLen = 12.4, secondLinkLen = 11, firstLinkX, firstLinkY, secondLinkX, secondLinkY;
		int angleKp[6] = {10, 20, 13, 1};
		int velocityKp[6] = {1, 1, 1, 1, 1, 1};
		const int motorAddress [6][2] = {{129, 6}, {129, 7}, {128, 7}, {130, 6}, {130, 7}, {128, 6}};
		float angle[4], angleSP[4];
		float wristHoldAngle;
		float wristAngleFromVert;
		int oldMode;
		int grabSpeed;
		float errorRatio;
};

#endif
