//Arm class implementation

#include "ros/ros.h"
#include "cui_encoders/arm_pos_enc_vals.h"
#include <qset_msgs/TargetAngle.h>
#include "Arm.h"
#include <std_msgs/Bool.h>

//TODO create getters and setter (hopfully this fixes the error that looks like a memory leak)
//
Arm::Arm() { 
	this->firstLinkLen = 12.4;
	this->secondLinkLen = 9;
}

void Arm::initializeAngleSP()
{
	this->angleSP[0] = this->angle[0];
	this->angleSP[1] = this->angle[1];
	this->angleSP[2] = this->angle[2];
	this->angleSP[3] = this->angle[3];
}

void Arm::joystickCallback(const qset_msgs::ArmControl msg) {
	this->controlMode = msg.armmode;
	this->joystickSP = msg.setpoint;
	if(msg.armmode == arm_control::ArmControl::SHOULDER){}
}

void Arm::angleCallback(const cui_encoders::arm_pos_enc_vals msg) {
	//TODO have andrew make these message members "elbow" and "shoulder" instead of "pos"
	this->angle[0] = msg.pan;
	this->angle[1] = msg.shoulder;
	this->angle[2] = msg.elbow;
	this->angle[3] = msg.wrist;
}

void Arm::angleSPCallback(const qset_msgs::TargetAngle msg) {
	this->angleSP[3] = msg.wrist;
	this->angleSP[2] = msg.elbow;
	this->angleSP[1] = msg.shoulder;
	this->angleSP[0] = msg.base;
}

void Arm::angleActiveCallback(const std_msgs::Bool msg) {
	if(msg.data == true){
		this->controlMode = -1;//deactivate all other control modes while angle control is active
	}
}

void Arm::stopAllJoints(qset_msgs::sabertooth _msg, ros::Publisher pub, Arm& _arm){
	this->joystickSP = 0;
	this->initializeAngleSP();
	ros::Rate send_rate(50);
	for(int i=0;i<6;i++){
		_msg.data = 64;
		_msg.address = _arm.motorAddress[i][0];
		_msg.command = _arm.motorAddress[i][1];
		pub.publish(_msg);
		send_rate.sleep();
	}
}

//TODO fix param list setting
/*
void Arm::paramSetting(const ros::NodeHandle n){
	//angleKP params
	n.param("pan_AngleKp", this->angleKp[0], this.angleKp[0]);
	n.param("shoulder_AngleKp", this->angleKp[1], this.angleKp[1]);
	n.param("elbow_AngleKp", this->angleKp[0], this.angleKp[2]);
	n.param("actuator_AngleKp", this->angleKp[3], this.angleKp[3]);
	n.param("wrist1_AngleKp", this->angleKp[4], this.angleKp[4]);
	n.param("wrist2_AngleKp", this->angleKp[5], this.angleKp[5]);

	//velocity Kp params
	n.param("pan_VelocityKp", this->velocityKp[0], this.velocityKp[0]);
	n.param("shoulder_VelocityKp", this->velocityKp[1], this.velocityKp[1]);
	n.param("elbow_VelocityKp", this->velocityKp[0], this.velocityKp[2]);
	n.param("actuator_VelocityKp", this->velocityKp[3], this.velocityKp[3]);
	n.param("wrist1_VelocityKp", this->velocityKp[4], this.velocityKp[4]);
	n.param("wrist2_VelocityKp", this->velocityKp[5], this.velocityKp[5]);

	//link lengths
	n.param("firstLinkLength", this->firstLinkLen, this.firstLinkLen);
	n.param("secondLinkLength", this->secondLinkLen, this.secondLinkLen);

}*/
