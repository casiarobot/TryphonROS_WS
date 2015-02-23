#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <ctime>

#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>
#include <ctime>


int main(int argc, char **argv)
{
double percent;
percent=atoi(argv[1]);
double impulse_time = atoi(argv[2]);
int dir = atoi(argv[3]);
int count=0;
ros::init(argc,argv,"command");
ros::NodeHandle n;

ros::Publisher wrench_publisher = n.advertise<geometry_msgs::Wrench>("/192_168_10_244/command_control",1);
ros::Rate loop_rate(20);


double force = -(double)2*percent/100;
geometry_msgs::Wrench wrenchMsg;

		wrenchMsg.force.x =0; //determine max thruster force
		wrenchMsg.force.y = 0;
		wrenchMsg.force.z = 0;
		wrenchMsg.torque.x = 0;
		wrenchMsg.torque.y = 0;
		wrenchMsg.torque.z = 0;

  if(dir==1){wrenchMsg.force.x=force;}
  if(dir==2){wrenchMsg.force.y=force;}
  if(dir==3){wrenchMsg.force.z=force;}

/*
clock_t begin;
clock_t temp;
clock_t end;
double elapsed_time;
double temp_info;
begin=clock();
*/


while (count<20 * impulse_time)
	{
		wrench_publisher.publish(wrenchMsg);
		++count;
		ros::spinOnce();
	        loop_rate.sleep();
		//temp=clock();
		//temp_info=double(temp-begin)/CLOCKS_PER_SEC;
		//ROS_INFO("time: %f",temp_info);
	}

ROS_INFO("Done:%f ",wrenchMsg.force.x);
/*end=clock();
elapsed_time=double(end-begin)/CLOCKS_PER_SEC*1000;
ROS_INFO("Time:%f\n", elapsed_time);*/

                wrenchMsg.force.x = 0; //determine max thruster force
                wrenchMsg.force.y = 0;
                wrenchMsg.force.z = 0;
                wrenchMsg.torque.x = 0;
                wrenchMsg.torque.y = 0;
                wrenchMsg.torque.z = 0;

while (count<25 * impulse_time)
        {
                wrench_publisher.publish(wrenchMsg);
                ++count;
                ros::spinOnce();
                loop_rate.sleep();
                //temp=clock();
                //temp_info=double(temp-begin)/CLOCKS_PER_SEC;
                //ROS_INFO("time: %f",temp_info);
        }
ROS_INFO("Done:%f ",force);

return 0;
}
