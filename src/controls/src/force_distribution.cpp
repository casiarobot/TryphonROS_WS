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

#include <dynamic_reconfigure/server.h>
#include <controls/fdistributionConfig.h>

#include "std_msgs/Float64.h"

bool start=true;
geometry_msgs::Wrench input, fz;
tests::props_command props;


// Parameters //
double maxForce=0.63;
double minForce=-0.32;
double L=1.115;
double Cm=0.17;
double maxPrct;


void callback(controls::fdistributionConfig &config, uint32_t level) {
  {
  ROS_INFO("Reconfigure Request: Max props percentage: %f",
           config.maxPrct);

  maxPrct=(config.maxPrct)/100;
  }
}

void subMax(const std_msgs::Float64 inputMsg)
{
  maxPrct=(inputMsg.data)/100;
}

void subForces(const geometry_msgs::Wrench inputMsg)
{
  input.force.x = inputMsg.force.x;
  input.force.y = inputMsg.force.y;
  input.force.z = inputMsg.force.z;
  input.torque.x = inputMsg.torque.x;
  input.torque.y = inputMsg.torque.y;
  input.torque.z = inputMsg.torque.z;
  start=false;
}

void subForcez(const geometry_msgs::Wrench inputMsg)
{
  fz.force.x = inputMsg.force.x;
  fz.force.y = inputMsg.force.y;
  fz.force.z = inputMsg.force.z;
  fz.torque.x = inputMsg.torque.x;
  fz.torque.y = inputMsg.torque.y;
  fz.torque.z = inputMsg.torque.z;
}

double Nocrash(double f1,double f0) //  TO TEST !!!!!!!!!!!!!!! //
{
  double f;
  if(fabs(f1-f0)>30)
  {
    if(f1>f0){f=f0+30;}
    else{f=f0-30;}
  }
  else f=f1;
  return f;

}


double RealForce(double f)
{
    double force=f;
    if(f>0)
    {
      if(fabs(force)<maxForce*0.05){force=0;}
      if(f>maxForce) {force=maxForce;}
    }
    else
    {
      if(fabs(force)<-minForce*0.05){force=0;}
      if(f<minForce) {force=minForce;}
    }

  return force;
}

double force2command(double f)
{

    double command;
    if(f>0)
    {
        if(f>maxForce)
        {
            return 235;
        }
        else
        {
            command=247.6222-sqrt(fabs(61316.75-92984.43*(f+0.0175878)));
            if(command<0){command=0;}
            return command;
        }
    }
    else
    {
        if(f< minForce)
        {
            return -233;
        }
        else
        {
            command=-244.3941+sqrt(fabs(59728.499+181679.79*(f-0.00951542)));
            if(command>0){command=0;}
            return command;
        }
    }
}


void Scale(double (&vP)[8][3])
{
   //assume that there is at least one propeller
  double best,ratio;
  if(vP[0][2]>0){best=vP[0][2]/maxForce;}
  else{best=vP[0][2]/minForce;}

  for(int k=1; k<8;k++) // looking for the biggest ratio among the propellers
  {
    if(vP[k][2]>0){ratio=vP[k][2]/maxForce;}
    else{ratio=vP[k][2]/minForce;}
    if(ratio>best){best=ratio;}
  }

  if(best>1)
  {
    for(int k=0; k<8;k++) // looking for the biggest ratio among the propellers
    {
      vP[k][2]=vP[k][2]/best;
    }
  }
}

void Smooth(double (&vP)[8][3],double (&vPraw)[8][3])
{
  for(int i=0;i<8;i++)
  {
    vP[i][2]=1.16826*vP[i][1]-0.42411820*vP[i][0]+0.0639643*vPraw[i][2]+0.127929*vPraw[i][1]+0.0639643*vPraw[i][0];
  }
}

double Saturation(double command)
{
  if(command>255.0*maxPrct){command=255.0*maxPrct;}
  if(command<-255.0*maxPrct){command=-255.0*maxPrct;}
  return command;

}


void UpdateProps(double (&vP)[8][3])
{
  for(int i=0; i<8;i++)
  {
    for(int k=0; k<2;k++)
    {
      vP[i][k]=vP[i][k+1];
    }
  }
}

ros::Publisher props_node;

void mySigintHandler(int sig)
{
  // Do some custom action.
  // For example, publish a stop message to some other nodes.
  for(int i=0; i<8;i++)
  {
    props.commands[i]=0;
  }
  props.header.stamp=ros::Time::now();

  props_node.publish(props);
  ROS_INFO("Done");
  // All the default sigint handler does is call shutdown()
  ros::shutdown();
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
  sprintf(rosname,"force_estimation_%s",get_ip());
  std::string s, temp_arg ;
  signal(SIGINT, mySigintHandler);

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

  dynamic_reconfigure::Server<controls::fdistributionConfig> server;
  dynamic_reconfigure::Server<controls::fdistributionConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  // Publishers //
  sprintf(rosname,"/%s/command_props",temp_arg.c_str());
  props_node = node.advertise<tests::props_command>(rosname,1);


  // Subscribers //
  sprintf(rosname,"/%s/max_thrust",temp_arg.c_str());
  ros::Subscriber subM = node.subscribe(rosname, 1, subMax);
  sprintf(rosname,"/%s/command_control",temp_arg.c_str());
  ros::Subscriber subF = node.subscribe(rosname, 1, subForces);
  sprintf(rosname,"/%s/intFz_control",temp_arg.c_str());
  ros::Subscriber subFz = node.subscribe(rosname, 1, subForcez);

  // Variables //
  double vP[8][3], vPraw[8][3], vPz[8][3], vPzraw[8][3], commands[8][3];  // only the last row of commands will be used but it is made so the function scale can be used.

  for(int i=0; i<8;i++)
  {
    for(int k=0; k<3;k++)
    {
      vP[i][k]=0;
      vPraw[i][k]=0;
      vPz[i][k]=0;
      vPzraw[i][k]=0;
      commands[i][k]=0;
    }
  }


  for(int i=0; i<8; i++)
  {
    props.commands.push_back(0);
  }

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    ros::spinOnce();

    if(!start)
    {

      vPraw[0][2]=((double)(input.force.x/2.000000-input.torque.z/(4.000000*L)));  // x right
      vPraw[1][2]=((double)(input.force.x/2.000000+input.torque.z/(4.000000*L)));  // x left
      vPraw[2][2]=((double)(-input.force.y/2.000000+input.torque.z/(4.000000*L)));  // y front
      vPraw[3][2]=((double)(-input.force.y/2.000000-input.torque.z/(4.000000*L)));  // y back
      vPraw[4][2]=((double)(input.force.z/4.000000+(-input.torque.x-input.torque.y-(L-Cm)*((input.force.x)-(input.force.y)))/(4.000000*L)));   // z front left
      vPraw[5][2]=((double)(input.force.z/4.000000+(-input.torque.x+input.torque.y-(L-Cm)*(-(input.force.x)-(input.force.y)))/(4.000000*L)));  // z front right
      vPraw[6][2]=((double)(input.force.z/4.000000+(input.torque.x+input.torque.y-(L-Cm)*(-(input.force.x)+(input.force.y)))/(4.000000*L)));   // z back left
      vPraw[7][2]=((double)(input.force.z/4.000000+(input.torque.x-input.torque.y-(L-Cm)*((input.force.x)+(input.force.y)))/(4.000000*L)));  // z back right

      vPzraw[0][2]=((double)(fz.force.x/2.000000-fz.torque.z/(4.000000*L)));  // x right
      vPzraw[1][2]=((double)(fz.force.x/2.000000+fz.torque.z/(4.000000*L)));  // x left
      vPzraw[2][2]=((double)(-fz.force.y/2.000000+fz.torque.z/(4.000000*L)));  // y front
      vPzraw[3][2]=((double)(-fz.force.y/2.000000-fz.torque.z/(4.000000*L)));  // y back
      vPzraw[4][2]=((double)(fz.force.z/4.000000+(-fz.torque.x-fz.torque.y-(L-Cm)*((fz.force.x)-(fz.force.y)))/(4.000000*L)));   // z front left
      vPzraw[5][2]=((double)(fz.force.z/4.000000+(-fz.torque.x+fz.torque.y-(L-Cm)*(-(fz.force.x)-(fz.force.y)))/(4.000000*L)));  // z front right
      vPzraw[6][2]=((double)(fz.force.z/4.000000+(fz.torque.x+fz.torque.y-(L-Cm)*(-(fz.force.x)+(fz.force.y)))/(4.000000*L)));   // z back left
      vPzraw[7][2]=((double)(fz.force.z/4.000000+(fz.torque.x-fz.torque.y-(L-Cm)*((fz.force.x)+(fz.force.y)))/(4.000000*L)));  // z back right

      Scale(vPraw);
      Scale(vPzraw);  // Normally useless but it's a protection

      Smooth(vP,vPraw);
      Smooth(vPz,vPzraw);

      UpdateProps(vPraw);
      UpdateProps(vP);

      UpdateProps(vPzraw);
      UpdateProps(vPz);

      for(int i=0; i<8;i++)
      {
          commands[i][2]=vP[i][2]+vPz[i][2];
      }

      for(int i=0; i<8;i++)
      {
        props.commands[i]=Saturation(Nocrash(force2command(RealForce(commands[i][2])),props.commands[i]));
      }
      props.header.stamp=ros::Time::now();

      props_node.publish(props);
    }
    loop_rate.sleep();
  }

  return 0;
}
