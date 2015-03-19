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



void subForces(const geometry_msgs::Wrench w)
{


}


ros::Publisher props_node;

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
  signal(SIGINT, mySigintHandler);
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
  sprintf(rosname,"/%s/command_props",temp_arg.c_str());
  props_node = node.advertise<tests::props_command>(rosname,1);


  // Subscribers //
  sprintf(rosname,"/%s/command_control",temp_arg.c_str());
  ros::Subscriber subF = node.subscribe(rosname, 1, subForces);


  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
