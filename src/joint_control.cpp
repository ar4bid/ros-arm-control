#include "Arm.h"
#include "joint_control.h"

#define StopCommand 64;
using namespace std;

int jointControl(float SP, Arm& _arm)
{
	float motorVal = SP*64 + 64.0;
	if(motorVal>_arm.maxSpeed)
	{
		motorVal = _arm.maxSpeed;
	} 
	else if(motorVal < _arm.minSpeed)
	{
		motorVal = _arm.minSpeed;
	}
	return (int)motorVal;
}

