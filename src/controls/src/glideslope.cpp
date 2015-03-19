
#include "glideslope.h"
#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include <Eigen/Geometry>
#include <Eigen/Dense>

ros::Publisher Glideslope;

geometry_msgs::Pose pi,pd;

void subPose(const geometry_msgs::PoseStamped PoseS)
{
pi=PoseS.pose;

pos(1)=pi
}


int main(int argc, char **argv)
{

char rosname[100],ip[100];
sprintf(rosname,"glideslope_%s",get_ip());
std::string s, temp_arg;
ros::init(argc, argv, rosname);
ros::NodeHandle node;
if (argc==2)
  {
    ROS_INFO("TARGET IS: %s", argv[1]);
  }
  else
  {
    ROS_ERROR("Failed to get param 'target'");
    return 0;
  }



 sprintf(rosname,"/%s/glideslope_dpos",temp_arg.c_str());
 Glideslope = node.advertise<geometry_msgs::Pose>(rosname,1);

 ros::Subscriber subP = node.subscribe("/ekf_node/pose", 1, subPose);


double a:





	return;
}