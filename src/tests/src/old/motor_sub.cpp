#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include "ros/ros.h"

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"

int a = 0;
int Arr[3];

void arrayCallback(const std_msgs::Int32MultiArray::ConstPtr& array)
{

        int i = 0;
        // print all the remaining numbers
        for(std::vector<int>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
        {
                Arr[i] = *it;
                i++;

        }
//        if (a>500){
        ROS_INFO("fx: %d,fy: %d,fz: %d",Arr[0],Arr[1],Arr[2]);
//        a=0;
//        }
//        else {a=a+1;}


        return;
}


int main(int argc, char **argv)
{

	ros::init(argc, argv, "motor_sub");

	ros::NodeHandle n;

	ros::Subscriber subar = n.subscribe("array", 1000, arrayCallback);

	ros::spin();

	return 0;
}

