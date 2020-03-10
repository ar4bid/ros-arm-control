#include "Arm.h"
#include "angle_control.h"

#define StopCommand 64;
 
using namespace std;


//TODO determine what needs to be read in through param-lists

float angleControl(int joint, Arm& _arm)
{
	float error = _arm.angle[joint] - _arm.angleSP[joint];
	if(joint == int(arm_control::ArmControl::ELBOW))
		error = error * -1;
	float motorVal = _arm.angleKp[joint] * error + 64.0; //This is simple PID, plus 64 to put it in the right range
	
	if(motorVal>_arm.maxSpeed)
		motorVal = _arm.maxSpeed;
	else if(motorVal<_arm.minSpeed)
		motorVal = _arm.minSpeed;
	
	int motorSpeed = motorVal; //type casting because the sabertooths require an int
	return(motorSpeed);
}
