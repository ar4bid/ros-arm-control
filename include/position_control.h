#ifndef POSITION_CONTROL_H
#define POSITION_CONTROL_H

#include "Arm.h"

void updateTargetAngles(Arm&);
void updateCurrentPos(Arm&);
float getWristAngle(Arm&);

#endif
