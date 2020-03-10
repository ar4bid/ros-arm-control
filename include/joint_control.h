#ifndef VOLTAGE_CONTROL_H
#define VOLTAGE_CONTROL_H

#include "Arm.h"

int jointControl(float, Arm&);
int wristControl(int, Arm&);
int grabControl(Arm&);
#endif
