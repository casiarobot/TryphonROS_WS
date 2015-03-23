#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <ctime>

#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>
#include <ctime>

#include "tests/props_command.h"


int main(int argc, char **argv)
{
  int count=0;
  ros::init(argc,argv,"test_motors");
  ros::NodeHandle n;

  ros::Publisher wrench_publisher = n.advertise<tests::props_command>("manual_control",1);
  ros::Rate loop_rate(2);


  tests::props_command props;
  for(int i=0; i<8;i++)
  {
    props.commands.push_back(0);
  }
  props.commands[0]=100;

  int prct=0;
  int prop=0;
  ROS_INFO("prop nb: %i",prop);

  while (ros::ok() && prop<8)
  {
    if(prct<231)
    {
      prct+=20;
      props.commands[prop]=prct;
    }
    else
    {
      prct=0;
      props.commands[prop]=prct;
      ++prop;
      ROS_INFO("prop nb: %i",prop);
    }
    wrench_publisher.publish(props);
    loop_rate.sleep();
  }


  prct=0;
  prop=0;
    while (ros::ok() && prop<8)
  {
    if(prct<231)
    {
      prct+=20;
      props.commands[prop]=-prct;
    }
    else
    {
      prct=0;
      props.commands[prop]=-prct;
      ++prop;
      ROS_INFO("prop nb: %i",prop);
    }
    wrench_publisher.publish(props);
    loop_rate.sleep();
  }
  ROS_INFO("Done");

  return 0;
}
