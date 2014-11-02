#include <ros/ros.h>
#include <stdlib.h>
#include "std_msgs/Int32.h"

int main(int argc, char ** argv)
{

std_msgs::Int32 i;


if (argc!=0)
i.data=atoi(argv[1]);
else
i.data=0;

ros::init(argc,argv,"modes");
ros::NodeHandle n;
ros::Rate loop_rate(20);
ros::Publisher mod_pub=n.advertise<std_msgs::Int32>("mode",1);

while(ros::ok())
{
	mod_pub.publish(i);
	loop_rate.sleep();
}
return 0;
}
