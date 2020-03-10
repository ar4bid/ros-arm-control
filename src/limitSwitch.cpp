
#include <stdio.h>
#include <wiringPi.h>
#include "ros/ros.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/Empty.h"
using namespace std;

int[] pinNumbers = {2, 3, 4}; // replace these with actual pin numbers
bool atLimit = false, allAtLimit = false; 
int main(void)
{
    if (wiringPiSetup() == -1)
    { //checks if wiringPi is working properly
        ROS_INFO("WiringPi is not set up propery, terminating limit switches node");
    }

    for (int i = 0; i < 3; i++)
        {
            pinMode(pinNumbers[i], INPUT);
        }

    ros::init(argc, argv, "limitSwitch");
    ros::NodeHandle n;
    ros::Publisher pub;
    ros::Publisher zero;
    pub = n.advertise<std_msgs::Int16MultiArray>("limitSwitch", 1);
    zero = n.advertise<std_msgs::Empty>("arm_control/zero_encoders",1);
    ros::Rate loop_rate(80);
    while (ros::ok())
    {

        std_msgs::Int16MultiArray data;
        for (int i = 0; i < 3; i++)
        {
	    allAtLimit = true;
            if(digitalRead(pinNumbers[i]) == 0){
                data.data[i]=1;
                atLimit = true;

            }else{
                data.data[i] = 0;
		allAtLimit = false;
            }
        }

        if(allAtLimit){
	    std_msgs::Empty = empty;
	    zero.publish(empty);
        }
        if(atLimit)
            pub.publish(data);

        atLimit = false;
        ros::spinOnce();
        loop_rate.sleep();
    }
}
