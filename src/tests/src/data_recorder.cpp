#include "ros/ros.h"
#include "sensors/sonar.h"
#include "sensors/sonarArray.h"
#include "sensors/compass.h"
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/Imu.h>
#include "sensors/imuros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"

#include <fstream>

#include <iostream>

using namespace std;

geometry_msgs::Pose PoseEKF,PoseCtrl,PoseDesir;
geometry_msgs::Twist VelEKF;
geometry_msgs::Wrench CmdCtrl,CmdReal;
sensor_msgs::Imu Imu;
double debut;

std::ofstream filePEKF;
std::ofstream fileVEKF;
std::ofstream fileCCtrl;
std::ofstream fileCRl;
std::ofstream filePCtrl;
std::ofstream filePD;
std::ofstream fileI;
std::ofstream filePath;

void subPoseEkf(const geometry_msgs::PoseStamped PoseS)
{
	PoseEKF=PoseS.pose;
	double secs = ros::Time::now().toSec()-debut;
	filePEKF <<secs << "," << PoseEKF.position.x << ","<< PoseEKF.position.y <<","<< PoseEKF.position.z <<","<< PoseEKF.orientation.x;
	filePEKF <<","<< PoseEKF.orientation.y <<","<< PoseEKF.orientation.z << "," << PoseEKF.orientation.w <<endl;
}

void subVelEKF(const geometry_msgs::TwistStamped VelS)
{
	VelEKF=VelS.twist;
	double secs = ros::Time::now().toSec()-debut;
	fileVEKF <<secs << "," << VelEKF.linear.x << ","<< VelEKF.linear.y <<","<< VelEKF.linear.z <<","<< VelEKF.angular.x;
	fileVEKF <<","<< VelEKF.angular.y <<","<< VelEKF.angular.z <<endl;
}

void subCommandControl(const geometry_msgs::Wrench inputMsg)
{
	CmdCtrl=inputMsg;
	double secs = ros::Time::now().toSec()-debut;
	fileCCtrl <<secs << "," << CmdCtrl.force.x << ","<< CmdCtrl.force.y <<","<< CmdCtrl.force.z <<","<< CmdCtrl.torque.x;
	fileCCtrl <<","<< CmdCtrl.torque.y <<","<< CmdCtrl.torque.z <<endl;
}

void subCommandReal(const geometry_msgs::WrenchStamped inputMsg)
{
	CmdReal=inputMsg.wrench;
	double secs = ros::Time::now().toSec()-debut;
	fileCRl <<secs << "," << CmdReal.force.x << ","<< CmdReal.force.y <<","<< CmdReal.force.z <<","<< CmdReal.torque.x;
	fileCRl <<","<< CmdReal.torque.y <<","<< CmdReal.torque.z <<endl;
}

void subPoseControl(const geometry_msgs::Pose Pose)
{
	PoseCtrl=Pose;
	double secs = ros::Time::now().toSec()-debut;
	filePCtrl <<secs << "," << PoseCtrl.position.x << ","<< PoseCtrl.position.y <<","<< PoseCtrl.position.z <<","<< PoseCtrl.orientation.x;
	filePCtrl <<","<< PoseCtrl.orientation.y <<","<< PoseCtrl.orientation.z << "," << PoseCtrl.orientation.w <<endl;
}

void subPoseDesir(const geometry_msgs::Pose Pose)
{
	PoseDesir=Pose;
	double secs = ros::Time::now().toSec()-debut;
	filePD <<secs << "," << PoseDesir.position.x << ","<< PoseDesir.position.y <<","<< PoseDesir.position.z <<","<< PoseDesir.orientation.x;
	filePD <<","<< PoseDesir.orientation.y <<","<< PoseDesir.orientation.z << "," << PoseDesir.orientation.w << endl;
}

void subImu(const sensor_msgs::Imu ImuValue)
{
	Imu=ImuValue;
	double secs = ros::Time::now().toSec()-debut;
	fileI <<secs << "," << Imu.linear_acceleration.x << ","<< Imu.linear_acceleration.y <<","<< Imu.linear_acceleration.z <<","<< Imu.angular_velocity.x;
	fileI <<","<< Imu.angular_velocity.y <<","<< Imu.angular_velocity.z <<endl;
}

void subPath(const geometry_msgs::Pose2D Pose)
{
	double secs = ros::Time::now().toSec()-debut;
	fileI <<secs << "," << Pose.x << ","<< Pose.y <<","<< Pose.theta  <<endl;
}

int main(int argc, char **argv)
{
   if (argc>1)
  {
    ROS_INFO("TARGET IS: %s", argv[1]);
    if (argc==3)
    {
    	ROS_INFO("File path is: %s", argv[2]);
    }
    else
    {
    	ROS_ERROR("Failed to get param 'file name' ");
    	return 0;
    }
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
  sprintf(rosname,"data_recorder_%s",temp_arg.c_str());


  ros::init(argc, argv, rosname);
  ros::NodeHandle node;

  // Subscribers //
  ros::Subscriber subPE = node.subscribe("/ekf_node/pose", 100, subPoseEkf);
  ros::Subscriber subV = node.subscribe("/ekf_node/velocity", 100, subVelEKF);
  sprintf(rosname,"/%s/command_control",temp_arg.c_str());
  ros::Subscriber subCC = node.subscribe(rosname, 100, subCommandControl);
  ros::Subscriber subCR = node.subscribe("tryphon/thrust", 100, subCommandReal);
  sprintf(rosname,"/%s/pose",temp_arg.c_str());
  ros::Subscriber subPC = node.subscribe(rosname, 100, subPoseControl);
  sprintf(rosname,"/%s/desired_pose",temp_arg.c_str());
  ros::Subscriber subPD = node.subscribe(rosname, 100, subPoseDesir);
  sprintf(rosname,"/%s/raw_imu",temp_arg.c_str());
  ros::Subscriber subI = node.subscribe(rosname, 100, subImu);
  sprintf(rosname,"/%s/path_info",temp_arg.c_str());
  ros::Subscriber subP = node.subscribe(rosname, 100, subPath);

  ros::Rate loop_rate(100);

  temp_arg = argv[2];
  char buffer[100];
  char link[100]="/home/tryphon/Rosbags/Gazebo_09_03_15";

  sprintf(buffer,"%s/%s_PEKF.csv",link,temp_arg.c_str());
  filePEKF.open(buffer);
  ROS_INFO(buffer);
  sprintf(buffer,"%s/%s_VEKF.csv",link,temp_arg.c_str());
  fileVEKF.open(buffer);
  ROS_INFO(buffer);
  sprintf(buffer,"%s/%s_CCtrl.csv",link,temp_arg.c_str());
  fileCCtrl.open(buffer);
  ROS_INFO(buffer);
  sprintf(buffer,"%s/%s_CRl.csv",link,temp_arg.c_str());
  fileCRl.open(buffer);
  ROS_INFO(buffer);
  sprintf(buffer,"%s/%s_PCtrl.csv",link,temp_arg.c_str());
  filePCtrl.open(buffer);
  ROS_INFO(buffer);
  sprintf(buffer,"%s/%s_PD.csv",link,temp_arg.c_str());
  filePD.open(buffer);
  ROS_INFO(buffer);
  sprintf(buffer,"%s/%s_I.csv",link,temp_arg.c_str());
  fileI.open(buffer);
  ROS_INFO(buffer);
  sprintf(buffer,"%s/%s_Path.csv",link,temp_arg.c_str());
  filePath.open(buffer);
  ROS_INFO(buffer);

  filePEKF   << "time,x,y,z,qx,qy,qz,qw" << endl  ;
  fileVEKF   << "time,vx,vy,vz,wx,wy,wz" << endl  ;
  fileCCtrl  << "time,fx,fy,fz,tx,ty,tz" << endl  ;
  fileCRl    << "time,fx,fy,fz,tx,ty,tz" << endl  ;
  filePCtrl  << "time,x,y,z,qx,qy,qz,qw" << endl  ;
  filePD     << "time,x,y,z,qx,qy,qz,qw" << endl  ;
  fileI      << "time,ax,ay,az,gx,gy,gz" << endl  ;
  filePath   << "time,pathNb,step,path"  << endl  ;



  debut = ros::Time::now().toSec();
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;

}



