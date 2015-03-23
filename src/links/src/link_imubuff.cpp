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

ros::Publisher imu_node;


int main(int argc, char **argv)
{
   if (argc==2)
  {
    ROS_INFO("TARGET IS: %s", argv[1]);
  }
  else
  {
    ROS_ERROR("Failed to get param 'target'");
    return 0;
  }
  char rosname[100],ip[100];
  std::string s, temp_arg ;
  temp_arg = argv[1];
  std::replace(temp_arg.begin(), temp_arg.end(), '.', '_');
  sprintf(rosname,"link_Imu_%s",temp_arg.c_str());


  ros::init(argc, argv, rosname);
  ros::NodeHandle node;

  // Publishers //
  sprintf(rosname,"/%s/raw_imu",temp_arg.c_str());
  imu_node = node.advertise<sensor_msgs::Imu>("/raw_imu",1);

  // Subscribers //
  sprintf(rosname,"/%s/imubuff",temp_arg.c_str());
  ros::Subscriber subI = node.subscribe(rosname, 1, subImu);

  ros::Rate loop_rate(200);
  ros::Duration dodo(0.005);

  int size =0;

  while (ros::ok())
  {
    ros::spinOnce();
    size=vImu.size();
    sensor_msgs::Imu Imu;

    if(needPublish)
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
