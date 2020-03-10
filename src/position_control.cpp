#include "cui_encoders/arm_pos_enc_vals.h"
#include <qset_msgs/ArmControl.h>
#include <qset_msgs/TargetAngle.h>
#include "qset_msgs/sabertooth.h"
#include <sensor_msgs/JointState.h>
#include <string.h>
#include <ros/ros.h>
#include "Arm.h"
#include "position_control.h"

#define StopCommand 64
#define PI 3.14159265359

using namespace std;

//If you want to understand the trig in either of these functions you should work it out yourself

//Don't operate this node when the elbow angle is past 180, or else bad news things will happen. This shouldn't happen anyways and we should add something that prevents this.	
void updateCurrentPos(Arm& _arm) 
{
	_arm.firstLinkX = _arm.firstLinkLen*sin(PI*_arm.angle[1]/180 - PI);
	_arm.firstLinkY = _arm.firstLinkLen*cos(PI*_arm.angle[1]/180 - PI);
	_arm.secondLinkX = _arm.secondLinkLen*cos((PI/2) + PI*(_arm.angle[2] - _arm.angle[1])/180);
	_arm.secondLinkY = _arm.secondLinkLen*sin((PI/2) + PI*(_arm.angle[2] - _arm.angle[1])/180);
	_arm.x = _arm.firstLinkX + _arm.secondLinkX;
	_arm.y = _arm.firstLinkY + _arm.secondLinkY;
}

void updateTargetAngles(Arm& _arm) 
{
	_arm.angleSP[2] = acos((pow(_arm.x,2) + pow(_arm.y,2) - pow(_arm.firstLinkLen,2) - pow(_arm.secondLinkLen,2))/(-2*_arm.firstLinkLen*_arm.secondLinkLen));	
	_arm.angleSP[1] = 3*PI/2 - atan2(_arm.y,_arm.x) - acos((pow(_arm.secondLinkLen,2) - pow(_arm.firstLinkLen,2) - pow(_arm.x,2) - pow(_arm.y,2))/(-2*_arm.firstLinkLen*sqrt(pow(_arm.x,2)+pow(_arm.y,2)))); 
	_arm.angleSP[1] = 180*_arm.angleSP[1]/PI;	
	_arm.angleSP[2] = 180*_arm.angleSP[2]/PI;
	_arm.angleSP[3] = _arm.wristHoldAngle - _arm.angleSP[2] + 90 + _arm.angleSP[1];
	for(int i=0; i<4; i++){
		if(_arm.angleSP[i] >= 360)
			_arm.angleSP[i] = _arm.angleSP[i] - 360;	
	}
}

float getWristAngle(Arm& _arm)
{
	return -_arm.angle[1]+ _arm.angle[2]+ _arm.angle[3] - 90;

}


