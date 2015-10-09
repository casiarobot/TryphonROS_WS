#include <math.h>
#include <ctime>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensors/compass.h>
#include "links/imuros.h"
#include "links/imubuff.h"

void subComp(const sensors::compass::ConstPtr& msg)
{
	ROS_INFO_STREAM("Yaw:" << msg->rz[0]);
}

void subImu(const links::imubuff Imus)
{
	double roll, pitch;
	double gx = Imus.buffer[0].accel[0];
	double gy = Imus.buffer[0].accel[1];
	double gz = Imus.buffer[0].accel[2];
	roll = atan2(gy,gz) * 180.0/M_PI;
	pitch = atan2(-gx,sqrt(gy*gy+gz*gz)) * 180.0/M_PI;

	ROS_INFO_STREAM("R:" << roll << " P:" << pitch);
}




int main(int argc, char **argv)
{
  ros::init(argc, argv, "links_imubuff");
  ros::NodeHandle nh("~");


  // Getting the parameters //
  std::string ip;
  if (nh.getParam("ip", ip))
  {
    ROS_INFO("links_imubuff started with address: %s", ip.c_str());
  }
  else
  {
    ROS_FATAL("Failed to get the ip");
    ros::shutdown();
  }

  char rosname[100];

  // Subscribers //
  sprintf(rosname,"/%s/imubuff",ip.c_str());
  ros::Subscriber subI = nh.subscribe(rosname, 1, subImu);
  // Subscribers //
  sprintf(rosname,"/%s/compass",ip.c_str());
  ros::Subscriber subC = nh.subscribe(rosname, 1, subComp);

  ros::Rate loop_rate(200);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;

}
