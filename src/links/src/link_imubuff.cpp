#include <math.h>
#include <ctime>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include "links/imuros.h"
#include "links/imubuff.h"

std::vector<sensor_msgs::Imu> vImu;
double deltaT=0.01;
bool needPublish=false;
double linCov[9]={0.01,0,0,0,0.01,0,0,0,0.01};
double aLinCov[9]={0.1,0,0,0,0.1,0,0,0,0.1};

void subImu(const links::imubuff Imus)
{
  vImu.clear();
  ros::Time now  = Imus.header.stamp;
  int       size = Imus.buffer.size();
  sensor_msgs::Imu Imu;

  for(int i=0; i<size; i++)
  {
    Imu.header.stamp=now-ros::Duration((double)(size-(i+1))*deltaT);
    Imu.linear_acceleration.x = Imus.buffer[i].accel[0];
    Imu.linear_acceleration.y = -Imus.buffer[i].accel[1]; // modifying the frame as such as the x-direction is given by the usb dongle and the z-direction is pointing upward
    Imu.linear_acceleration.z = -Imus.buffer[i].accel[2];

    Imu.angular_velocity.x = (Imus.buffer[i].gyro[0])*0.017453; // modifying the frame as such as the x-direction is given by the usb dongle and the z-direction is pointing upward
    Imu.angular_velocity.y = (-Imus.buffer[i].gyro[1])*0.017453;
    Imu.angular_velocity.z = (-Imus.buffer[i].gyro[2])*0.017453;

    for(int i=0; i<9;i++)
    {
      Imu.linear_acceleration_covariance[i]=linCov[i];
      Imu.angular_velocity_covariance[i]=aLinCov[i];
    }

    vImu.push_back(Imu);

  }
  needPublish=true;
}




int main(int argc, char **argv)
{
  ros::init(argc, argv, "link_imubuff");
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
  // Publishers //
  sprintf(rosname,"/%s/raw_imu",ip.c_str());
  ros::Publisher imu_node = nh.advertise<sensor_msgs::Imu>(rosname,1);

  // Subscribers //
  sprintf(rosname,"/%s/imubuff",ip.c_str());
  ros::Subscriber subI = nh.subscribe(rosname, 1, subImu);

  ros::Rate loop_rate(200);
  ros::Duration dodo(0.005);

  int size =0;

  while (ros::ok())
  {
    ros::spinOnce();
    size=vImu.size();
    sensor_msgs::Imu Imu;

    if(needPublish) // we don't want to republish twice the same data
    {
      //ROS_INFO("size of vImu: %i",size);
      for(int i=0; i<size; i++)
      {
        //ROS_INFO("i: %i",i);
        imu_node.publish(vImu[i]);
        dodo.sleep();
      }
      needPublish=false;
    }

    loop_rate.sleep();
  }

  return 0;

}
