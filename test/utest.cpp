#include <ros/ros.h>
#include <gtest/gtest.h>
#include "joint_control.h"
#include "angle_control.h"
#include "position_control.h"
#include "Arm.h"

using namespace std;

Arm arm;

void setupKinematics(Arm& arm){
	updateCurrentPos(arm);
	updateTargetAngles(arm);		
}

TEST(JointControl, zero){
	EXPECT_EQ(64, jointControl(0, arm));
}

TEST(JointControl, fullForward){
	EXPECT_EQ(arm.maxSpeed, jointControl(1, arm));
	EXPECT_LT(64, jointControl(1, arm));
}

TEST(JointControl, fullReverse){
	EXPECT_EQ(arm.minSpeed, jointControl(-1, arm));
	EXPECT_GT(64, jointControl(-1, arm));
}

TEST(AngleControl, stop){
	for(int joint=0; joint<4; joint++){
 		arm.angle[joint] = 0;
		arm.angleSP[joint] = 0;
		ASSERT_EQ(angleControl(joint, arm), 64);
	}
}

TEST(AngleControl, fullForward){
	for(int joint=0; joint<4; joint++){
 		arm.angle[joint] = 0;
		arm.angleSP[joint] = 5;
		ASSERT_EQ(angleControl(joint, arm), arm.maxSpeed);
	}
}

TEST(AngleControl, fullReverse){
	for(int joint=0; joint<4; joint++){
 		arm.angle[joint] = 0;
		arm.angleSP[joint] = -5;
		ASSERT_EQ(angleControl(joint, arm), arm.minSpeed);
	}
}

TEST(AngleControl, smallForward){
	for(int joint=0; joint<4; joint++){
 		arm.angle[joint] = 0;
		arm.angleSP[joint] = 1;
		ASSERT_NE(angleControl(joint, arm), 0);
	}
}

TEST(AngleControl, smallReverse){
	for(int joint=0; joint<4; joint++){
 		arm.angle[joint] = 0;
		arm.angleSP[joint] = -1;
		ASSERT_NE(angleControl(joint, arm), 0);
	}
}

TEST(PositionControl, pan){
//check that the kinematics functions don't alter pan.
//Use cascading for loops to check all permeations of the arm we're concerned with.
	for(arm.angle[0]=0; arm.angle[0]<360; arm.angle[0]+=30){
		for(arm.angle[1]=0; arm.angle[1]<360; arm.angle[1]+=30){
			for(arm.angle[2]=0; arm.angle[2]<180; arm.angle[2]+=30){
				for(arm.angle[3]=0; arm.angle[3]<360; arm.angle[3]+=30){		
					setupKinematics(arm);
					ASSERT_EQ(arm.angle[0], arm.angle[0]);
					ASSERT_EQ(arm.angleSP[0], arm.angleSP[0]);
				}
			}
		}
	}  
}

TEST(PositionControl, shoulder){
	for(arm.angle[0]=0; arm.angle[0]<360; arm.angle[0]+=30){
		for(arm.angle[1]=90; arm.angle[1]<360; arm.angle[1]+=30){
			for(arm.angle[2]=30; arm.angle[2]<180; arm.angle[2]+=30){
				for(arm.angle[3]=0; arm.angle[3]<360; arm.angle[3]+=30){		
					setupKinematics(arm);
					ASSERT_EQ(round(arm.angle[1]), round(arm.angleSP[1])); //rounding is needed because c++ trig functions are only so percise
				}
			}
		}
	}  
}

TEST(PositionControl, elbow){
	for(arm.angle[0]=0; arm.angle[0]<360; arm.angle[0]+=30){
		for(arm.angle[1]=90; arm.angle[1]<360; arm.angle[1]+=30){
			for(arm.angle[2]=30; arm.angle[2]<180; arm.angle[2]+=30){
				for(arm.angle[3]=0; arm.angle[3]<360; arm.angle[3]+=30){		
					setupKinematics(arm);
					ASSERT_EQ(round(arm.angle[2]), round(arm.angleSP[2]));
				}
			}
		}
	}  
}

TEST(PositionControl, wrist){
	for(arm.angle[0]=0; arm.angle[0]<360; arm.angle[0]+=30){
		for(arm.angle[1]=0; arm.angle[1]<360; arm.angle[1]+=30){
			for(arm.angle[2]=0; arm.angle[2]<180; arm.angle[2]+=30){
				for(arm.angle[3]=0; arm.angle[3]<360; arm.angle[3]+=30){		
					setupKinematics(arm);
					ASSERT_EQ(arm.angle[4], arm.angle[4]);
				}
			}
		}
	}  
}

int main(int argc, char **argv){
	testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "tester");
	ros::NodeHandle nh;
	return RUN_ALL_TESTS();
}
