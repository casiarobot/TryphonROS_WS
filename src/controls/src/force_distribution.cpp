#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <ctime>

#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>
#include <ctime>

#include "tests/props_command.h"
#include "sensors/motorArray.h"

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
double nbMotor=0;


void callback(controls::fdistributionConfig &config, uint32_t level) {
  {
  ROS_INFO("Reconfigure Request: Max props percentage: %f",
           config.maxPrct);

  maxPrct=(config.maxPrct)/100.000;
  }
}

void subMax(const std_msgs::Float64 inputMsg)
{
  maxPrct=(inputMsg.data)/100.000;
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

void subMotor(const sensors::motorArray mArray)
{
  nbMotor=mArray.motors.size();
  for(int i=0; i<nbMotor; ++i)
  {
    if(mArray.motors[i].volt<6.0){ROS_WARN("Motor %02X has low battery",mArray.motors[i].id*2);}
  }
}

double Nocrash(double f1,double f0)
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


/*
double RealForce(double f)
{
    double force=f;
    if(f>0)
    {
      if(f>maxForce) {force=maxForce;}
    }
    else
    {
      if(f<minForce) {force=minForce;}
    }

  return force;
}
*/

double AvoidNoise(double c)
{
	if(fabs(c)<10){c=0;}	
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
            if(command<0 || command==0){command=0;}
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
            if(command>0 || command==0){command=0;}
            return command;
        }
    }
}


void Scale(double (&vP)[12][3])
{
   //assume that there is at least one propeller
  double best,ratio;
  if(vP[0][2]>0){best=vP[0][2]/maxForce;}
  else{best=vP[0][2]/minForce;}

  for(int k=1; k<12;k++) // looking for the biggest ratio among the propellers
  {
    if(vP[k][2]>0){ratio=vP[k][2]/maxForce;}
    else{ratio=vP[k][2]/minForce;}
    if(ratio>best){best=ratio;}
  }

  if(best>1)
  {
    for(int k=0; k<12;k++) // applying the scaling
    {
      vP[k][2]=vP[k][2]/best;
    }
  }
}

void Smooth(double (&vP)[12][3],double (&vPraw)[12][3])
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


void UpdateProps(double (&vP)[12][3])
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

int main(int argc, char **argv)
{

 
  signal(SIGINT, mySigintHandler);

  ros::init(argc, argv, "force_distribution");
  ros::NodeHandle node;

  dynamic_reconfigure::Server<controls::fdistributionConfig> server;
  dynamic_reconfigure::Server<controls::fdistributionConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  // Publishers //
  props_node = node.advertise<tests::props_command>("command_props",1);


  // Subscribers //
 
  ros::Subscriber subM = node.subscribe("max_thrust", 1, subMax);
  ros::Subscriber subF = node.subscribe("command_control", 1, subForces);
  ros::Subscriber subFz = node.subscribe("intFz_control", 1, subForcez);
  ros::Subscriber subMI = node.subscribe("motors_info", 1, subMotor);

  // Variables //
  double vP[12][3], vPraw[12][3], vPz[12][3], vPzraw[12][3], commands[12][3];  // only the last row of commands will be used but it is made so the function scale can be used.

  for(int i=0; i<12;i++)
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


  for(int i=0; i<12; i++)
  {
    props.commands.push_back(0);
  }

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    ros::spinOnce();

    if(!start)
    {
      if(nbMotor<9)
      {
        vPraw[0][2]=((double)(input.force.x/2.000000+input.torque.z/(4.000000*L)));  // x right
        vPraw[1][2]=((double)(input.force.x/2.000000-input.torque.z/(4.000000*L)));  // x left
        vPraw[2][2]=((double)(input.force.y/2.000000+input.torque.z/(4.000000*L)));  // y front
        vPraw[3][2]=((double)(input.force.y/2.000000-input.torque.z/(4.000000*L)));  // y back
        vPraw[4][2]=((double)(input.force.z/4.000000+(input.torque.x-input.torque.y-(L-Cm)*((input.force.x)+(input.force.y)))/(4.000000*L)));   // z front left
        vPraw[5][2]=((double)(input.force.z/4.000000+(-input.torque.x-input.torque.y-(L-Cm)*((input.force.x)-(input.force.y)))/(4.000000*L)));  // z front right
        vPraw[6][2]=((double)(input.force.z/4.000000+(-input.torque.x+input.torque.y-(L-Cm)*(-(input.force.x)-(input.force.y)))/(4.000000*L)));   // z back left
        vPraw[7][2]=((double)(input.force.z/4.000000+(input.torque.x+input.torque.y-(L-Cm)*(-(input.force.x)+(input.force.y)))/(4.000000*L)));  // z back right
        vPraw[8][2]=0;  // x top right
        vPraw[9][2]=0;  // x top left
        vPraw[10][2]=0;  // y top front
        vPraw[11][2]=0;  // y top back

        vPzraw[0][2]=((double)(fz.force.x/2.000000+fz.torque.z/(4.000000*L)));  // x right
        vPzraw[1][2]=((double)(fz.force.x/2.000000-fz.torque.z/(4.000000*L)));  // x left
        vPzraw[2][2]=((double)(fz.force.y/2.000000+fz.torque.z/(4.000000*L)));  // y front
        vPzraw[3][2]=((double)(fz.force.y/2.000000-fz.torque.z/(4.000000*L)));  // y back
        vPzraw[4][2]=((double)(fz.force.z/4.000000+(fz.torque.x-fz.torque.y-(L-Cm)*((fz.force.x)+(fz.force.y)))/(4.000000*L)));   // z front left
        vPzraw[5][2]=((double)(fz.force.z/4.000000+(-fz.torque.x-fz.torque.y-(L-Cm)*((fz.force.x)-(fz.force.y)))/(4.000000*L)));  // z front right
        vPzraw[6][2]=((double)(fz.force.z/4.000000+(-fz.torque.x+fz.torque.y-(L-Cm)*(-(fz.force.x)-(fz.force.y)))/(4.000000*L)));   // z back left
        vPzraw[7][2]=((double)(fz.force.z/4.000000+(fz.torque.x+fz.torque.y-(L-Cm)*(-(fz.force.x)+(fz.force.y)))/(4.000000*L)));  // z back right
        vPzraw[8][2]=0;  // x top right
        vPzraw[9][2]=0;  // x top left
        vPzraw[10][2]=0;  // y top front
        vPzraw[11][2]=0;  // y top back
      }
      else
      {
		vPraw[0][2]=((double)(input.force.x/4.000000+(-input.torque.y+input.torque.z)/(8.000000*L)));  // x right
        vPraw[1][2]=((double)(input.force.x/4.000000+(-input.torque.y-input.torque.z)/(8.000000*L)));  // x left
        vPraw[2][2]=((double)(input.force.y/4.000000+(input.torque.x+input.torque.z)/(8.000000*L)));  // y front
        vPraw[3][2]=((double)(input.force.y/4.000000+(input.torque.x-input.torque.z)/(8.000000*L)));  // y back
        vPraw[4][2]=((double)(input.force.z/4.000000+(input.torque.x-input.torque.y)/(8.000000*L)));   // z front left
        vPraw[5][2]=((double)(input.force.z/4.000000+(-input.torque.x-input.torque.y)/(8.000000*L)));  // z front right
        vPraw[6][2]=((double)(input.force.z/4.000000+(-input.torque.x+input.torque.y)/(8.000000*L)));   // z back left
        vPraw[7][2]=((double)(input.force.z/4.000000+(input.torque.x+input.torque.y)/(8.000000*L)));  // z back right
        vPraw[8][2]=((double)(input.force.x/4.000000+(input.torque.y+input.torque.z)/(8.000000*L)));  // x top right
        vPraw[9][2]=((double)(input.force.x/4.000000+(input.torque.y-input.torque.z)/(8.000000*L)));;  // x top left
        vPraw[10][2]=((double)(input.force.y/4.000000+(-input.torque.x+input.torque.z)/(8.000000*L)));  // y top front
        vPraw[11][2]=((double)(input.force.y/4.000000+(-input.torque.x-input.torque.z)/(8.000000*L)));  // y top back

        vPzraw[0][2]=(double)(fz.force.x/4.000000);  // x right
        vPzraw[1][2]=(double)(fz.force.x/4.000000);  // x left
        vPzraw[2][2]=(double)(fz.force.y/4.000000);  // y front
        vPzraw[3][2]=(double)(fz.force.y/4.000000);  // y back
        vPzraw[4][2]=(double)(fz.force.z/4.000000);   // z front left
        vPzraw[5][2]=(double)(fz.force.z/4.000000);  // z front right
        vPzraw[6][2]=(double)(fz.force.z/4.000000);   // z back left
        vPzraw[7][2]=(double)(fz.force.z/4.000000);  // z back right
        vPzraw[8][2]=(double)(fz.force.x/4.000000);  // x top right
        vPzraw[9][2]=(double)(fz.force.x/4.000000);  // x top left
        vPzraw[10][2]=(double)(fz.force.y/4.000000);  // y top front
        vPzraw[11][2]=(double)(fz.force.y/4.000000);  // y top back
        /*
        vPraw[0][2]=((double)(input.force.x/4.000000-input.torque.z/(4.000000*L)));  // x right
        vPraw[1][2]=((double)(input.force.x/4.000000+input.torque.z/(4.000000*L)));  // x left
        vPraw[2][2]=((double)(input.force.y/4.000000-input.torque.z/(4.000000*L)));  // y front
        vPraw[3][2]=((double)(input.force.y/4.000000+input.torque.z/(4.000000*L)));  // y back
        vPraw[4][2]=((double)(input.force.z/4.000000+(-input.torque.x-input.torque.y)/(4.000000*L)));   // z front left
        vPraw[5][2]=((double)(input.force.z/4.000000+(-input.torque.x+input.torque.y)/(4.000000*L)));  // z front right
        vPraw[6][2]=((double)(input.force.z/4.000000+(input.torque.x+input.torque.y)/(4.000000*L)));   // z back left
        vPraw[7][2]=((double)(input.force.z/4.000000+(input.torque.x-input.torque.y)/(4.000000*L)));  // z back right
        vPraw[8][2]=((double)(input.force.x/4.000000+input.torque.z/(4.000000*L)));  // x top right
        vPraw[9][2]=((double)(input.force.x/4.000000-input.torque.z/(4.000000*L)));  // x top left
        vPraw[10][2]=((double)(input.force.y/4.000000+input.torque.z/(4.000000*L)));  // y top front
        vPraw[11][2]=((double)(input.force.y/4.000000-input.torque.z/(4.000000*L)));  // y top back
        

        vPzraw[0][2]=((double)(fz.force.x/4.000000+(-fz.torque.z)/(4.000000*L)));  // x right
        vPzraw[1][2]=((double)(fz.force.x/4.000000+(+fz.torque.z)/(4.000000*L)));  // x left
        vPzraw[2][2]=((double)(fz.force.y/4.000000+(-fz.torque.z)/(4.000000*L)));  // y front
        vPzraw[3][2]=((double)(fz.force.y/4.000000+(+fz.torque.z)/(4.000000*L)));  // y back
        vPzraw[4][2]=((double)(fz.force.z/4.000000+(-fz.torque.x-fz.torque.y)/(4.000000*L)));   // z front left
        vPzraw[5][2]=((double)(fz.force.z/4.000000+(-fz.torque.x+fz.torque.y)/(4.000000*L)));  // z front right
        vPzraw[6][2]=((double)(fz.force.z/4.000000+(fz.torque.x+fz.torque.y)/(4.000000*L)));   // z back left
        vPzraw[7][2]=((double)(fz.force.z/4.000000+(fz.torque.x-fz.torque.y)/(4.000000*L)));  // z back right
        vPzraw[8][2]=((double)(fz.force.x/4.000000+(+fz.torque.z)/(4.000000*L)));  // x top right
        vPzraw[9][2]=((double)(fz.force.x/4.000000+(-fz.torque.z)/(4.000000*L)));  // x top left
        vPzraw[10][2]=((double)(fz.force.y/4.000000+(+fz.torque.z)/(4.000000*L)));  // y top front
        vPzraw[11][2]=((double)(fz.force.y/4.000000+(-fz.torque.z)/(4.000000*L)));  // y top back*/
      }

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

      for(int i=0; i<12;i++)
      {
        //props.commands[i]=AvoidNoise(Saturation(Nocrash(force2command(RealForce(commands[i][2])),props.commands[i])));
        props.commands[i]=Saturation(Nocrash(force2command(RealForce(commands[i][2])),props.commands[i]));
      }
      props.header.stamp=ros::Time::now();

      props_node.publish(props);
    }
    loop_rate.sleep();
  }

  return 0;
}
