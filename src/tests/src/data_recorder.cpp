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
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <sensor_msgs/Imu.h>
#include "sensors/imuros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "tests/props_command.h"
#include "sensors/imubuff.h"
#include <fstream>
#include <Euler_utility.h>
#include <iostream>

using namespace std;

geometry_msgs::Pose PoseEKF,PoseGaz,PoseCtrl,PoseDesir, Posetraj, PoseAR1, PoseIacc, PoseIgyr,PoseRP,PoseARconv;
geometry_msgs::Twist VelEKF, VelGAZ;
geometry_msgs::Wrench CmdCtrl,CmdReal;
sensor_msgs::Imu Imu;
double debut;

std::ofstream filePAR1;
std::ofstream fileIMUB;
std::ofstream fileIMUPOSE;
std::ofstream fileARC;

std::ofstream filePGAZ;
std::ofstream fileVGAZ;
std::ofstream filePEKF;
std::ofstream fileVEKF;

std::ofstream filetraj;
std::ofstream fileCCtrl;
std::ofstream fileCRl;
std::ofstream filePCtrl;
std::ofstream filePD;
std::ofstream fileI;
std::ofstream filePath;
std::ofstream fileCP;
std::ofstream fileSICKA;
std::ofstream fileSICKB;

void subIMUB(const sensors::imubuff::ConstPtr& msg)
{
 double roll, pitch;
  double accel[3];
  accel[0]=msg->buffer[0].accel[0];
  accel[1]=-msg->buffer[0].accel[1];
  accel[2]=-msg->buffer[0].accel[2];
  EulerU::getRollPitchIMU(accel,roll,pitch);
double secs = ros::Time::now().toSec()-debut;
fileIMUB << secs << "," << roll <<","<< pitch <<endl;

}

void subIMUPose(const geometry_msgs::PoseStamped PoseIMU)
{
  PoseRP=PoseIMU.pose;
  double secs = ros::Time::now().toSec()-debut;
  fileIMUPOSE <<secs << "," << PoseRP.position.x << ","<< PoseRP.position.y <<","<< PoseRP.position.z <<","<< PoseRP.orientation.x;
  fileIMUPOSE <<","<< PoseRP.orientation.y <<","<< PoseRP.orientation.z << "," << PoseRP.orientation.w <<endl;
}

void subARconv(const geometry_msgs::PoseStamped PoseARC)
{
  PoseARconv=PoseARC.pose;
  double secs = ros::Time::now().toSec()-debut;
  fileARC <<secs << "," << PoseARconv.position.x << ","<< PoseARconv.position.y <<","<< PoseARconv.position.z <<","<< PoseARconv.orientation.x;
  fileARC <<","<< PoseARconv.orientation.y <<","<< PoseARconv.orientation.z << "," << PoseARconv.orientation.w <<endl;
}


void subARtag1(const ar_track_alvar_msgs::AlvarMarkers Alvar1)
{
 
  ar_track_alvar_msgs::AlvarMarker alvartemp;
  alvartemp=Alvar1.markers[0];
  PoseAR1=alvartemp.pose.pose ; 
  double secs = ros::Time::now().toSec()-debut;
  filePAR1 <<secs << "," << PoseAR1.position.x << ","<< PoseAR1.position.y <<","<< PoseAR1.position.z <<","<< PoseAR1.orientation.x;
  filePAR1 <<","<< PoseAR1.orientation.y <<","<< PoseAR1.orientation.z << "," << PoseAR1.orientation.w <<endl;
}

void subPoseGaz(const geometry_msgs::PoseStamped PoseSG)
{
  PoseGaz=PoseSG.pose;
  double secs = ros::Time::now().toSec()-debut;
  filePGAZ <<secs << "," << PoseGaz.position.x << ","<< PoseGaz.position.y <<","<< PoseGaz.position.z <<","<< PoseGaz.orientation.x;
  filePGAZ <<","<< PoseGaz.orientation.y <<","<< PoseGaz.orientation.z << "," << PoseGaz.orientation.w <<endl;
}

void subVelGaz(const geometry_msgs::TwistStamped VelSG)
{
  VelGAZ=VelSG.twist;
  double secs = ros::Time::now().toSec()-debut;
  fileVGAZ <<secs << "," << VelGAZ.linear.x << ","<< VelGAZ.linear.y <<","<< VelGAZ.linear.z <<","<< VelGAZ.angular.x;
  fileVGAZ <<","<< VelGAZ.angular.y <<","<< VelGAZ.angular.z <<endl;
}

void subtrajdes(const geometry_msgs::Pose Pose)
{
  Posetraj=Pose;
  double secs = ros::Time::now().toSec()-debut;
  filetraj <<secs << "," << Posetraj.position.x << ","<< Posetraj.position.y <<","<< Posetraj.position.z <<","<< Posetraj.orientation.x;
  filetraj <<","<< Posetraj.orientation.y <<","<< Posetraj.orientation.z << "," << Posetraj.orientation.w <<endl;
} 

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

void subPoseDesir(const geometry_msgs::PoseStamped Pose)
{
  PoseDesir=Pose.pose;
  double secs = ros::Time::now().toSec()-debut;
  filePD <<secs << "," << PoseDesir.position.x << ","<< PoseDesir.position.y <<","<< PoseDesir.position.z <<","<< PoseDesir.orientation.x;
  filePD <<","<< PoseDesir.orientation.y <<","<< PoseDesir.orientation.z << "," << PoseDesir.orientation.w << endl;
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
	filePath <<secs << "," << Pose.x << ","<< Pose.y <<","<< Pose.theta  <<endl;
}

void subCommandProps(const tests::props_command props)
{
  double commands[8]={0,0,0,0,0,0,0,0};
  for(int i=0;i<props.commands.size();i++){commands[i]=props.commands[i];}
	double secs = ros::Time::now().toSec()-debut;
	fileCP <<secs << "," << commands[0] << ","<< commands[1] <<","<< commands[2] <<","<< commands[3];
	fileCP <<","<< commands[4] <<","<< commands[5] <<","<< commands[6] <<","<< commands[7] <<endl;
}

void subSICKA(const geometry_msgs::PoseStamped PoseS)
{
	PoseEKF=PoseS.pose;
	double secs = ros::Time::now().toSec()-debut;
	fileSICKA <<secs << "," << PoseEKF.position.x << ","<< PoseEKF.position.y <<","<< PoseEKF.position.z <<","<< PoseEKF.orientation.x;
	fileSICKA <<","<< PoseEKF.orientation.y <<","<< PoseEKF.orientation.z << "," << PoseEKF.orientation.w <<endl;
}

void subSICKB(const geometry_msgs::PoseStamped  PoseS)
{
	PoseEKF=PoseS.pose;
	double secs = ros::Time::now().toSec()-debut;
	fileSICKB <<secs << "," << PoseEKF.position.x << ","<< PoseEKF.position.y <<","<< PoseEKF.position.z <<","<< PoseEKF.orientation.x;
	fileSICKB <<","<< PoseEKF.orientation.y <<","<< PoseEKF.orientation.z << "," << PoseEKF.orientation.w <<endl;
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
  //ros::Subscriber subPE = node.subscribe("/ekf_node/pose", 100, subPoseEkf);
  //ros::Subscriber subV = node.subscribe("/ekf_node/velocity", 100, subVelEKF);

/////////////////////////////////ArTag recording

sprintf(rosname,"/%s/artags1/artag1/ar_pose_marker",temp_arg.c_str());
  ros::Subscriber subPARtag1 = node.subscribe(rosname, 100, subARtag1);
   sprintf(rosname,"/%s/imubuff",temp_arg.c_str()); //messed up Ip's
  ros::Subscriber subIMUBuff = node.subscribe(rosname, 100, subIMUB);
  sprintf(rosname,"/%s/imu_rp",temp_arg.c_str()); 
  ros::Subscriber subIMUPOSE = node.subscribe(rosname, 100, subIMUPose);
  sprintf(rosname,"/%s/pose_ar_frame",temp_arg.c_str()); 
  ros::Subscriber subarconv = node.subscribe(rosname, 100, subARconv);
 //////////////////////////////////////////////



/////////////////////////////////////Gazebo recording
  /*
  sprintf(rosname,"/%s/poseStamped_gazebo",temp_arg.c_str());
  ros::Subscriber subPGaz = node.subscribe(rosname, 100, subPoseGaz);
   sprintf(rosname,"/%s/velocity",temp_arg.c_str());
  ros::Subscriber subVGaz = node.subscribe(rosname, 100, subVelGaz);
  sprintf(rosname,"/%s/state_estimator/pose",temp_arg.c_str());
  ros::Subscriber subPE = node.subscribe(rosname, 100, subPoseEkf);
  sprintf(rosname,"/%s/state_estimator/vel",temp_arg.c_str());
  ros::Subscriber subV = node.subscribe(rosname, 100, subVelEKF);
  
  sprintf(rosname,"/%s/desired_pose",temp_arg.c_str());
  ros::Subscriber subPD = node.subscribe(rosname, 100, subPoseDesir);
  sprintf(rosname,"/%s/traj_data",temp_arg.c_str());
  ros::Subscriber supProp=node.subscribe(rosname,100,subtrajdes);
*/
/////////////////////////////////////

///////////////////////////Misc
  /*
  sprintf(rosname,"/%s/command_control",temp_arg.c_str());
  ros::Subscriber subCC = node.subscribe(rosname, 100, subCommandControl);
  ros::Subscriber subCR = node.subscribe("tryphon/thrust", 100, subCommandReal);
  sprintf(rosname,"/%s/pose",temp_arg.c_str());
  ros::Subscriber subPC = node.subscribe(rosname, 100, subPoseControl);
   sprintf(rosname,"/%s/raw_imu",temp_arg.c_str());
  ros::Subscriber subI = node.subscribe(rosname, 100, subImu);
  sprintf(rosname,"/%s/path_info",temp_arg.c_str());
  ros::Subscriber subP = node.subscribe(rosname, 100, subPath);
  sprintf(rosname,"/%s/command_props",temp_arg.c_str());
  ros::Subscriber subCP = node.subscribe(rosname, 100, subCommandProps);
  sprintf(rosname,"/%s/cubeA_pose",temp_arg.c_str());
  ros::Subscriber subSA = node.subscribe(rosname, 100, subSICKA);
  sprintf(rosname,"/%s/cubeB_pose",temp_arg.c_str());
  ros::Subscriber subSB = node.subscribe(rosname, 100, subSICKB);
  */
////////////////////////////////////

  ros::Rate loop_rate(100);

  temp_arg = argv[2];
  
  char buffer[100];
  char link[100]="/home/patrick/tryphon_data";

  

  /////////////////////////////////////////////////ArTag Recording
 sprintf(buffer,"%s/%s/%s_PAR1.csv",link,temp_arg.c_str(),argv[1]);
  filePAR1.open(buffer);
  ROS_INFO(buffer);
   sprintf(buffer,"%s/%s/%s_IMUB.csv",link,temp_arg.c_str(),argv[1]);
  fileIMUB.open(buffer);
  ROS_INFO(buffer);
  sprintf(buffer,"%s/%s/%s_IMUPose.csv",link,temp_arg.c_str(),argv[1]);
  fileIMUPOSE.open(buffer);
  ROS_INFO(buffer);
  sprintf(buffer,"%s/%s/%s_Arconv.csv",link,temp_arg.c_str(),argv[1]);
  fileARC.open(buffer);
  ROS_INFO(buffer);
//////////////////////////////////////////////////////////////////////

  /////////////////////////////////////////////////Gazebo Recording
  /*
  sprintf(buffer,"%s/%s/%s_PGAZ.csv",link,temp_arg.c_str(),argv[1]);
  filePGAZ.open(buffer);
  ROS_INFO(buffer);
   sprintf(buffer,"%s/%s/%s_VGAZ.csv",link,temp_arg.c_str(),argv[1]);
  fileVGAZ.open(buffer);
  ROS_INFO(buffer);
  sprintf(buffer,"%s/%s/%s_PEKF.csv",link,temp_arg.c_str(),argv[1]);
  filePEKF.open(buffer);
  ROS_INFO(buffer);
  sprintf(buffer,"%s/%s/%s_VEKF.csv",link,temp_arg.c_str(),argv[1]);
  fileVEKF.open(buffer);
  ROS_INFO(buffer);
   sprintf(buffer,"%s/%s/%s_PD.csv",link,temp_arg.c_str(),argv[1]);
  filePD.open(buffer);
  ROS_INFO(buffer);
 sprintf(buffer,"%s/%s/%s_traj.csv",link,temp_arg.c_str(),argv[1]);
  filetraj.open(buffer);
  ROS_INFO(buffer);
  */
//////////////////////////////////////////////////////////////////////
  

////////////////////////////////////////////////////////////////////////////Misc
  /*
  sprintf(buffer,"%s/%s/%s_CCtrl.csv",link,temp_arg.c_str(),argv[1]);
  fileCCtrl.open(buffer);
  ROS_INFO(buffer);
  sprintf(buffer,"%s/%s/%s_CRl.csv",link,temp_arg.c_str(),argv[1]);
  fileCRl.open(buffer);
  ROS_INFO(buffer);
  sprintf(buffer,"%s/%s/%s_PCtrl.csv",link,temp_arg.c_str(),argv[1]);
  filePCtrl.open(buffer);
  ROS_INFO(buffer);
   sprintf(buffer,"%s/%s/%s_I.csv",link,temp_arg.c_str(),argv[1]);
  fileI.open(buffer);
  ROS_INFO(buffer);
  sprintf(buffer,"%s/%s/%s_Path.csv",link,temp_arg.c_str(),argv[1]);
  filePath.open(buffer);
  ROS_INFO(buffer);
  sprintf(buffer,"%s/%s/%s_CP.csv",link,temp_arg.c_str(),argv[1]);
  fileCP.open(buffer);
  ROS_INFO(buffer);
  sprintf(buffer,"%s/%s/%s_SICKA.csv",link,temp_arg.c_str(),argv[1]);
  fileSICKA.open(buffer);
  ROS_INFO(buffer);
  sprintf(buffer,"%s/%s/%s_SICKB.csv",link,temp_arg.c_str(),argv[1]);
  fileSICKB.open(buffer);
  ROS_INFO(buffer);
*/
  ////////////////////////////////////////////////////////////////////////
  


/////////////////////////////////////////////////ArTag Recording
filePAR1   << "time,x,y,z,qx,qy,qz,qw" << endl  ;
fileIMUB   << "time,roll,pitch" << endl  ;
fileIMUPOSE   << "time,x,y,z,qx,qy,qz,qw" << endl  ;
fileARC   << "time,x,y,z,qx,qy,qz,qw" << endl  ;
//////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////Gazebo recording
/*
  filePGAZ   << "time,x,y,z,qx,qy,qz,qw" << endl  ;
  fileVGAZ   << "time,x,y,z,qx,qy,qz,qw" << endl  ;
  filePEKF   << "time,x,y,z,qx,qy,qz,qw" << endl  ;
  fileVEKF   << "time,vx,vy,vz,wx,wy,wz" << endl  ;
  filePD     << "time,x,y,z,qx,qy,qz,qw" << endl  ;
  filetraj   << "time,x,y,z,qx,qy,qz,qw" << endl  ;  
 */
////////////////////////////////////////////////////////////////////////
 
 ////////////////////////////////////////////////////////////////////////////Misc
  /*
  fileProp   << "time,x,y,z,qx,qy,qz,qw" << endl  ;
  fileCCtrl  << "time,fx,fy,fz,tx,ty,tz" << endl  ;
  fileCRl    << "time,fx,fy,fz,tx,ty,tz" << endl  ;
  filePCtrl  << "time,x,y,z,qx,qy,qz,qw" << endl  ;
    fileI      << "time,ax,ay,az,gx,gy,gz" << endl  ;
  filePath   << "time,pathNb,step,path"  << endl  ;
  fileCP     << "time,p0,p1,p2,p3,p4,p5,p6,p7" << endl  ;
  fileSICKA   << "time,x,y,z,qx,qy,qz,qw" << endl  ;
  fileSICKB   << "time,x,y,z,qx,qy,qz,qw" << endl  ;
*/
////////////////////////////////////////////////////////////////////////


  debut = ros::Time::now().toSec();
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;

}



