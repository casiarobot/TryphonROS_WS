#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <ctime>

#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>
#include <ctime>

#include "tests/props_command.h"

#include <vector>
#include <cmath>

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <net/if.h>
#include <arpa/inet.h>
#include <signal.h>
#include <iostream>

#include <Eigen/Geometry>
#include <Eigen/Dense>


ros::Publisher command_node;

void zero_wrench(geometry_msgs &w)
{
	w.force.x=0;
	w.force.y=0;
	w.force.z=0;
	w.torque.x=0;
	w.torque.y=0;
	w.torque.z=0;
}


const char* get_ip()
{
  int fd;
  struct ifreq ifr;
  char *ip = new char[100];

  fd = socket(AF_INET, SOCK_DGRAM, 0);

  /* I want to get an IPv4 IP address */
  ifr.ifr_addr.sa_family = AF_INET;

  /* I want IP address attached to "eth0" */
  strncpy(ifr.ifr_name, "eth0", IFNAMSIZ-1);

  ioctl(fd, SIOCGIFADDR, &ifr);

  close(fd);

  /* display result */
  sprintf(ip,"%s", inet_ntoa(((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr));
  std::string s = ip;
  std::replace(s.begin(), s.end(), '.', '_');
  //ip=s.c_str();
  return s.c_str();
}

int main(int argc, char **argv)
{

  char rosname[100],ip[100];
  sprintf(rosname,"control_%s",get_ip());
  std::string s, temp_arg ;

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

  temp_arg = argv[1];
  std::replace(temp_arg.begin(), temp_arg.end(), '.', '_');

  // Publishers //
  sprintf(rosname,"/%s/command_control",temp_arg.c_str());
  command_node = node.advertise<geometry_msgs>(rosname,1);

  double force[13]={-0.64,0.5,0.4,0.3,0.2,0.1,0,0.2,0.4,0.6,0.8,1.0,1.24};
  double torque[13]={-0.64,0.5,0.4,0.3,0.2,0.1,0,0.2,0.4,0.6,0.8,1.0,1.24};




  for(int i=0;i<13;i++)
  {
    wrench[1][i]=torque[i];
  }

  int j=0;
  int k=0;
  int l=0;

  geometry_msgs command;


  ros::Rate loop_rate(2);

  ROS_INFO("Test force");

  while(ros::ok() && k<2 )
  {
    zero_wrench(command);
    switch(j)
    {
      case 0: if(k<1){command.force.x=force[l];}
      else {command.torque.x=torque[l];}
      break;
      case 1: if(k<1){command.force.y=force[l];}
      else {command.torque.y=torque[l];}
      break;
      case 2: if(k<1){command.force.z=force[l];}
      else {command.torque.z=torque[l];}
      break;
    }

    command_node.publish(command);

    if(l<12){l++;}
    else
    {
      if(j<2)
      {
        j++;
        l=0;
      }
      else
      {
        if(k<2){ROS_INFO("Test force");}
        else{ROS_INFO("Done");}
        k++;
        j=0;
        l=0;
      }
    }
  }

  return 0;
}
