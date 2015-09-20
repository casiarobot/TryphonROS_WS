//library for ros
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>


// Standard C/C++ libraries
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <ctime>
#include <unistd.h>
#include <string.h> /* for strncpy */

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <net/if.h>
#include <arpa/inet.h>


// Eigen libraries
#include <Eigen/Geometry>
#include <Eigen/Dense>


// ROS messages libraries
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include <sensor_msgs/Imu.h>

//libraries for the sonars and the compass
#include "sensors/sonar.h"
#include "sensors/sonarArray.h"
#include "sensors/leddar.h"
#include "sensors/leddarArray.h"
#include "sensors/compass.h"
#include "sensors/imubuff.h"
#include "sensors/imuros.h"
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>
// YAML file
//#include <YAMLParser.h>

// Euler file
#include <Euler_utility.h>

// Kalman file
#include <Kalman_utility.h>




// Global variable //
double tIMU, tMCPTAM, tSICK, tSonars, tComp;
sensor_msgs::Imu Imu;

bool start_IMU=true, start_MCPTAM=true, start_CMP=true, start_sonar=true, start_leddar=true, start_SICK=true;
bool pose_received;

double dsxt[5]={0,0,0,0,0};
double dsyt[5]={0,0,0,0,0};
double dszt[5]={0,0,0,0,0};
double dsrt[5]={0,0,0,0,0};
double dsxtf[5]={0,0,0,0,0};
double dsytf[5]={0,0,0,0,0};
double dsztf[5]={0,0,0,0,0};
double dsrtf[5]={0,0,0,0,0};
double rz0=0;

Eigen::Vector3d SONARpos, SICKpos, angle, avel;
Eigen::VectorXd MCPTAMpos(6);
Eigen::Vector3d CMIMUpos(1,0,-1.125); // vector postion from Center of mass to IMU in tryphon frame
Eigen::Matrix3d Rmatrix, CPCMIMUmatrix, RIMUmatrix, CPCMCmatrix;
Eigen::Quaterniond quatIMU(0.99255, 0,0.12187, 0); // quat of the rotation matrix between the tryphon frame and the IMU frame


///////For Real tryphon///////
/*
Eigen::Quaterniond quatMCPTAM(0.64,-0.755, -0.065,0.066); // quat of the rotation matrix between the tryphon frame and the IMU frame
Eigen::Vector3d CMCAMpos(1.125,0,1.125); // vector postion from Center of mass to IMU in tryphon frame
Eigen::Vector3d GyroOffset(-0.183,-0.124,-1.40);
*/

/////////for Gazebo////////
Eigen::Vector3d CMCAMpos(0,0,0);
Eigen::Quaterniond quatMCPTAM(1,0,0,0);
Eigen::Vector3d GyroOffset(0,0,0);


Eigen::Matrix3d CPM(Eigen::Vector3d vect) // return cross product matrix
{
	Eigen::Matrix3d CPM;
	CPM<<       0,  -vect(2),   vect(1),
		  vect(2),         0,  -vect(0),
		  -vect(1),  vect(0),         0;

	return CPM;
}

//Publishers //


//Subscribers //
void subLeddar(const sensors::leddarArray::ConstPtr& msg)
{

	pose_received = true;
	double dsx = 0, dsy = 0, dsz = 0;
	for(int i = 0; i < 4;i++){
		dsxt[i] = dsxt[i+1];
		dsxtf[i] = dsxtf[i+1];
		dsyt[i] = dsyt[i+1];
		dsytf[i] = dsytf[i+1];
		dszt[i] = dszt[i+1];
		dsztf[i] = dsztf[i+1];
	}
	//dszto=dszt;
	for (int i=0; i < msg->leddars.size(); ++i)
	{
		const sensors::leddar &leddar = msg->leddars[i];
		//ROS_INFO_STREAM("ID: " << sonar.id << " - D0: " << sonar.distance[0] <<
		//	               ", D1: " << sonar.distance[1]);

		// From cm to m
		switch (leddar.id) {
			case 0x90:
				dsx = leddar.distance / 100.0;
				break;
			case 0x92:
				dsy = leddar.distance / 100.0;
				break;
			case 0x98:
				dsz = leddar.distance / 100.0;
				break;
		}

	}

	dsxt[4] = dsx;
	dsyt[4] = dsy;
	dszt[4] = dsz;

	// Sonars filtering
	dsxtf[4]=3.159*dsxtf[3]-3.815*dsxtf[2]+2.076*dsxtf[1]-0.4291*dsxtf[0]+0.01223*dsxt[4]-0.02416*dsxt[3]+0.03202*dsxt[2]-0.02416*dsxt[1]+0.01223*dsxt[0];
	dsytf[4]=3.159*dsytf[3]-3.815*dsytf[2]+2.076*dsytf[1]-0.4291*dsytf[0]+0.01223*dsyt[4]-0.02416*dsyt[3]+0.03202*dsyt[2]-0.02416*dsyt[1]+0.01223*dsyt[0];
	dsztf[4]=3.159*dsztf[3]-3.815*dsztf[2]+2.076*dsztf[1]-0.4291*dsztf[0]+0.01223*dszt[4]-0.02416*dszt[3]+0.03202*dszt[2]-0.02416*dszt[1]+0.01223*dszt[0];
	//ROS_INFO("distance z: %f, distance x : %f, distance y : %f",dsztf[4],dsxtf[4],dsytf[4]);

	//SONARpos = Eigen::Vector3d(dsxtf[4], dsytf[4], dsztf[4]);
	/*SONARpos(0) = dsxtf[4];
	SONARpos(1) = dsytf[4];
	SONARpos(2) = dsztf[4];*/

	if(start_leddar){
		start_leddar=false;
		ROS_INFO("Leddar started!");
	}
}

void subSonar(const sensors::sonarArray::ConstPtr& msg)
{
	pose_received = true;
	int hmax=6000;
	double dsz1=0, dsz2=0, dsz3=0, dsz4=0, dsx=0, dsy=0, okdsz=0;
	for(int i=0; i<4;i++){
		dsxt[i]=dsxt[i+1];
		dsxtf[i]=dsxtf[i+1];
		dsyt[i]=dsyt[i+1];
		dsytf[i]=dsytf[i+1];
		dszt[i]=dszt[i+1];
		dsztf[i]=dsztf[i+1];
	}
	//dszto=dszt;
	for (int i=0; i<msg->sonars.size(); ++i)
	{
		const sensors::sonar &sonar = msg->sonars[i];
		//ROS_INFO_STREAM("ID: " << sonar.id << " - D0: " << sonar.distance[0] <<
		//	               ", D1: " << sonar.distance[1]);

		// Average but not dividing by ten to convert cm into mm
		if (sonar.id == (int)(0xE0)/2){
			dsx = -sonar.distance / 100.000;
		}
		else if (sonar.id == (int)(0xE6)/2){
			dsy = sonar.distance / 100.000;
		}
		else if (sonar.id == (int)(0xF8)/2){
			dsz3 = sonar.distance / 100.000;
			if(dsz3>hmax)
				dsz3 = 0;
			else
				++okdsz;
		}
		else if (sonar.id == (int)(0xFA)/2){
			dsz4 = sonar.distance / 100.000;
			if(dsz4>hmax)
				dsz4 = 0;				
			else
				++okdsz;
		}
		else if (sonar.id == (int)(0xFC)/2){
			dsz1 = sonar.distance / 100.000;
			if(dsz1>hmax)
				dsz1 = 0;
			else
				++okdsz;
		}
		else if (sonar.id == (int)(0xFE)/2){
			dsz2 = sonar.distance / 100.000;
			if(dsz2>hmax)
				dsz2 = 0;
			else
				++okdsz;
		}

	}
	dsxt[4]=dsx;dsyt[4]=dsy;
	if(okdsz!=0)
		dszt[4]=(dsz1+dsz2+dsz3+dsz4)/okdsz;
	else
		dszt[4]=dszt[3];

	// Sonars filtering
	dsxtf[4]=3.159*dsxtf[3]-3.815*dsxtf[2]+2.076*dsxtf[1]-0.4291*dsxtf[0]+0.01223*dsxt[4]-0.02416*dsxt[3]+0.03202*dsxt[2]-0.02416*dsxt[1]+0.01223*dsxt[0];
	dsytf[4]=3.159*dsytf[3]-3.815*dsytf[2]+2.076*dsytf[1]-0.4291*dsytf[0]+0.01223*dsyt[4]-0.02416*dsyt[3]+0.03202*dsyt[2]-0.02416*dsyt[1]+0.01223*dsyt[0];
	dsztf[4]=3.159*dsztf[3]-3.815*dsztf[2]+2.076*dsztf[1]-0.4291*dsztf[0]+0.01223*dszt[4]-0.02416*dszt[3]+0.03202*dszt[2]-0.02416*dszt[1]+0.01223*dszt[0];

	//SONARpos = Eigen::Vector3d(dsxtf[4], dsytf[4], dsztf[4]);
	SONARpos(0) = dsxtf[4];
	SONARpos(1) = dsytf[4];
	SONARpos(2) = dsztf[4];

	if(start_sonar){
		start_sonar=false;
		ROS_INFO("Sonars started!");
	}

}

void subposeSick(geometry_msgs::PoseStamped ps){
  if(ps.pose.position.x==ps.pose.position.x)
    SICKpos(0)=ps.pose.position.x;
  if(ps.pose.position.y==ps.pose.position.y)
    SICKpos(1)=ps.pose.position.y;

  //rz2=-sgn(ps.pose.orientation.z)*ps.pose.orientation.w;
  Eigen::Quaterniond quad(ps.pose.orientation.x,ps.pose.orientation.y,ps.pose.orientation.z,ps.pose.orientation.w);
  float rz2_tmp = atan2(quad.toRotationMatrix()(1, 2), quad.toRotationMatrix()(2, 2));
  if(rz2_tmp == rz2_tmp) //Testing if NaN
    SICKpos(2)=rz2_tmp;
/*
  if ((last_rz2-rz2) < -1.6)
          rz2_init=rz2_init+3.1416;
  
  if ((last_rz2-rz2) > 1.6)
          rz2_init=rz2_init-3.1416;     

  float rz2_tmp = rz2 - rz2_init;
  
  if (rz2_tmp > 1.6)
          rz2_init=rz2_init+3.1416;     

  if (rz2_tmp < -1.6)
          rz2_init=rz2_init-3.1416;
  
  last_rz2 = rz2;
  rz2 = rz2 - rz2_init;*/
  //rz2=-3.14*sin(rz2);
  
    if(start_SICK)
  {
    start_SICK=false;
    ROS_INFO("SICK started!");
  }
}

void subComp(const sensors::compass::ConstPtr& msg)
{
    //ROS_INFO_STREAM("ID: " << msg->id << " - RZ0: " << msg->rz[0] <<
    //                ", RZ1: " << msg->rz[1]);
    int rz1=0;

    for(int i=0; i<4;i++){
        dsrt[i]=dsrt[i+1];
		dsrtf[i]=dsrtf[i+1];
	}

	if(msg->id == (int)(0xC0)/2)
		rz1=(msg->rz[0])-rz0;

	if (start_CMP && rz1!=0){
		rz0=rz1;
		start_CMP=false;
	}

    dsrt[4]=rz1;
    //ROS_INFO("rotation: %f",rz);

    dsrtf[4]=3.159*dsrtf[3]-3.815*dsrtf[2]+2.076*dsrtf[1]-0.4291*dsrtf[0]+0.01223*dsrt[4]-0.02416*dsrt[3]+0.03202*dsrt[2]-0.02416*dsrt[1]+0.01223*dsrt[0];
    angle(3)=dsrtf[4];
}

///////////////////For Real Tryphon////////////////////
/*
void subImu(const sensors::imubuff::ConstPtr& msg)
{
  double roll, pitch;
  double accel[3]={msg->buffer[0].accel[0],-msg->buffer[0].accel[1],-msg->buffer[0].accel[2]};
  EulerU::getRollPitchIMU(accel,roll,pitch);
  angle(0)=(angle(0)+roll)/2; // low pass filter
  angle(1)=(angle(1)-0.244+pitch)/2; // low pass filter

  Eigen::Vector3d avel_temp;
	avel_temp(0)=(msg->buffer[0].gyro[0]-GyroOffset(0))*0.017453;
	avel_temp(1)=-(msg->buffer[0].gyro[1]-GyroOffset(1))*0.017453;
	avel_temp(2)=-(msg->buffer[0].gyro[2]-GyroOffset(2))*0.017453;
	avel_temp=RIMUmatrix*avel_temp;
  avel_temp=EulerU::RbodyEuler(angle(0),angle(1))*avel_temp;
  avel=(avel+avel_temp)/2;

  if(start_IMU){
    start_IMU=false;
    ROS_INFO("IMU started!");
  }

	//mx=msg->buffer[0].magn[0];
	//my=msg->buffer[0].magn[1];
	//mz=msg->buffer[0].magn[2];
}*/

////////////For Gazebo////////////////////////
void subImu(const sensor_msgs::Imu Imu_msg)
{
  double roll, pitch;
  EulerU::getRollPitchIMU(Imu_msg,roll,pitch);
  angle(0)=(angle(0)+roll)/2; // low pass filter
  angle(1)=(angle(1)-0.244+pitch)/2; // low pass filter

  Eigen::Vector3d avel_temp;
  avel_temp(0)=Imu_msg.angular_velocity.x; // defined in IMU frame
  avel_temp(1)=Imu_msg.angular_velocity.y;
  avel_temp(2)=Imu_msg.angular_velocity.z;
  avel_temp=RIMUmatrix*avel_temp;  // defined in body frame

  avel_temp=EulerU::RbodyEuler(angle(0),angle(1))*avel_temp;
  avel=(avel+avel_temp)/2;
    if(start_IMU)
  {
    start_IMU=false;
    ROS_INFO("IMU started!");
  }

}
 Eigen::Quaterniond quatCAM3(0,0,-.5,1);

//void subMCPTAM(const ar_track_alvar_msgs::AlvarMarkers Aposes) // with MCPTAM
void subMCPTAM(const ar_track_alvar_msgs::AlvarMarkers Aposes)
{


  CMCAMpos(0)=-1.05;
  CMCAMpos(1)=1.125;
  CMCAMpos(2)=.17;

  tMCPTAM=ros::Time::now().toSec();
  pose_received=true;
  ar_track_alvar_msgs::AlvarMarker alvartemp;
  alvartemp=Aposes.markers[0];
  geometry_msgs::Pose Pose=alvartemp.pose.pose ; //Pose=Aposes.pose;
  Eigen::Vector3d MCPTAMposition;

  MCPTAMposition(0)=Pose.position.x; // defined in global frame
  MCPTAMposition(1)=Pose.position.y;
  MCPTAMposition(2)=Pose.position.z;
  Eigen::Quaterniond quat(Pose.orientation.w,Pose.orientation.x,Pose.orientation.y,Pose.orientation.z);
  //Eigen::Quaterniond quat1(Pose.orientation.w,Pose.orientation.x,Pose.orientation.y,Pose.orientation.z);
  //Eigen::Quaterniond quat2(Pose.orientation.w,Pose.orientation.x,Pose.orientation.y,Pose.orientation.z);
  //Eigen::Quaterniond quat3(Pose.orientation.w,Pose.orientation.x,Pose.orientation.y,Pose.orientation.z);
  
Eigen::Quaterniond quattf;

////////////For Real tryphon//////////// 
 /*
  EulerU::getQuatFromEuler(quattf, 0,0,-M_PI/2);
*/

///////For Gazebo////////////////
EulerU::getQuatFromEuler(quattf, 0,0,0);


  
  
  //quat=quat*quatMCPTAM.inverse()*quattf.inverse();  // compute the quaternion between the vision world and the tryphon frame
  quat=quat*quatCAM3.inverse()*quattf.inverse();  // compute the quaternion between the vision world and the tryphon frame




  MCPTAMpos(3)=atan2(2*(quat.w()*quat.x()+quat.y()*quat.z()),1-2*(quat.x()*quat.x()+quat.y()*quat.y()));
  MCPTAMpos(4)=asin(2*(quat.w()*quat.y()-quat.z()*quat.x()));
  MCPTAMpos(5)=atan2(2*(quat.w()*quat.z()+quat.x()*quat.y()),1-2*(quat.z()*quat.z()+quat.y()*quat.y()));
  Rmatrix=quat.toRotationMatrix();
  MCPTAMposition=MCPTAMposition-Rmatrix*CMCAMpos;  // offset due to the fact that the pose is the one of the IMU
  MCPTAMpos(0)=MCPTAMposition(0);
  MCPTAMpos(1)=MCPTAMposition(1);
  MCPTAMpos(2)=MCPTAMposition(2);
  
  if(start_MCPTAM)
  {
    start_MCPTAM=false;
    ROS_INFO("MCPTAM started");
  }
}


int main(int argc, char **argv)
{
    char rosname[100];
    std::string temp_arg;
    int pos_src=0;  //0=MCPTAM,1=SICK(+SONARZ),2=SONARS,3=LEDDARS ALL[+IMU,+CMP]

  ros::init(argc, argv, "state_estimator");
  ros::NodeHandle nh;
  ros::NodeHandle nh1("~");


std::string ip_address;
    if (nh1.getParam("ip", ip_address))
    {
      ROS_INFO("Got param: %s", ip_address.c_str());
    }
    else
    {
      ROS_ERROR("Failed to get param 'ip'");
      ros::shutdown();
    }

std::string pos_src_string;
  if (nh1.getParam("pos_src", pos_src))
    {
      ROS_INFO("Got pos_src: %d", pos_src);
      
    }
    else
    {
      ROS_ERROR("Failed to get param 'pos_src'");
    }

    


  // Loading from YAML files //
  Eigen::Vector3d rel_pos_CM, rel_pos_camera, rel_pos_IMU, rel_pos_SICK;
  Eigen::Quaterniond rel_quat_CM, rel_quat_camera, rel_quat_IMU, rel_quat_SICK;

  // Subscribers //
  /* sensors_thruster use imubuff, gazebo use raw_imu*/
  
  //////For Real Tryphon///////
  //ros::Subscriber subI = nh.subscribe("imubuff", 1, subImu);
  
  /////////For Gazebo////////////
 ros::Subscriber subI = nh.subscribe("raw_imu", 1, subImu); //raw_imu or imubuff

  ros::Subscriber subM = nh.subscribe("artags/artag1/ar_pose_marker",1,subMCPTAM);
 // ros::Subscriber subS = nh.subscribe("sonars", 1, subSonar);
 // ros::Subscriber subL = nh.subscribe("leddars", 1, subLeddar);
 // ros::Subscriber subC = nh.subscribe("compass",1, subComp);
  ros::Subscriber subSP = nh.subscribe("/cubeA_pose",1, subposeSick);

  //Publishers //
  ros::Publisher pubP = nh.advertise<geometry_msgs::PoseStamped>("state_estimator/pose",1);
  ros::Publisher pubV = nh.advertise<geometry_msgs::TwistStamped>("state_estimator/vel",1);

  


  static tf::TransformBroadcaster br;
  tf::Transform transform;

   int rate = 100;
  if(pos_src!=0 && pos_src!=1){
    rate = 10;
    start_MCPTAM=false;
  }
  ros::Rate loop_rate(rate);
  // variables //


  // Relatives frames variables
  RIMUmatrix=quatIMU.toRotationMatrix(); // need to be computed only once
  CPCMIMUmatrix=CPM(CMIMUpos);

  Eigen::Quaterniond quat;



  // Matrices and vectors for the kalman filter //
  Eigen::VectorXd xk1k1(12),xk1k(12),xkk(12);

  Eigen::MatrixXd F(12,12), Q(12,12), Pk1k1(12,12), Pk1k(12,12), Pkk(12,12),
		  I6(6,6), O6(3,3), I2(2,2), I3(3,3), O3(3,3), O2(2,2), I13(6,6), I12(6,6),
		  RIMU(5,5), RMCPTAM(6,6), RSONAR(3,3), RLEDDAR(3,3), RSICK(3,3);
  int obsStates=11;
  if(pos_src==3)
      obsStates=9;
  else if(pos_src==1)
      obsStates=10;
  Eigen::MatrixXd S(obsStates,obsStates), K(12,obsStates), H(obsStates,12), R(obsStates, obsStates);
  Eigen::VectorXd z(obsStates), y(obsStates); // measurement and measurement residual
  //Eigen::MatrixXd R(6,6), S(6,6), K(12,6), H(6,12);

  I2=Eigen::MatrixXd::Identity(2,2);
  I3=Eigen::MatrixXd::Identity(3,3);
  O3=Eigen::MatrixXd::Zero(3,3);
  O2=Eigen::MatrixXd::Zero(2,2);
  I6=Eigen::MatrixXd::Identity(6,6);
  O6=Eigen::MatrixXd::Zero(6,6);
  I12=Eigen::MatrixXd::Identity(12,12);

//Errors variance matrices
  RMCPTAM << 0.1*I3,O3,O3,0.005*I3;
  RSONAR << 0.1*I3;
  RLEDDAR << 0.1*I3;
  RSICK << 0.1*I3;
  float RCMP = 0.05;
  RIMU << 0.05*I2,Eigen::MatrixXd::Zero(2,3),Eigen::MatrixXd::Zero(3,2),0.05*I3;
  if(pos_src==0)
    R << RMCPTAM,Eigen::MatrixXd::Zero(6,5),
    Eigen::MatrixXd::Zero(5,6),RIMU;
  else if(pos_src==1)
    R << RSICK,Eigen::MatrixXd::Zero(3,7),
    Eigen::MatrixXd::Zero(1,3),0.1,Eigen::MatrixXd::Zero(1,6),
    Eigen::MatrixXd::Zero(1,4),RCMP,Eigen::MatrixXd::Zero(1,5),
    Eigen::MatrixXd::Zero(5,5),RIMU;
  else if(pos_src==3)
    R << RSONAR,Eigen::MatrixXd::Zero(3,6),Eigen::MatrixXd::Zero(1,3),RCMP,Eigen::MatrixXd::Zero(1,5),Eigen::MatrixXd::Zero(5,4),RIMU;
  //R << RMCPTAM;
ROS_INFO("Initialization2");
//State observation matrix
if(pos_src==0){
	  H << I6, O6,
	   Eigen::MatrixXd::Zero(2,3), I2, Eigen::MatrixXd::Zero(2,7),
	   Eigen::MatrixXd::Zero(3,9), I3;
} else if(pos_src==1) {
	  H << I3, Eigen::MatrixXd::Zero(3,9),
	   Eigen::MatrixXd::Zero(1,5), 1, Eigen::MatrixXd::Zero(1,6),
	   O3, I3, Eigen::MatrixXd::Zero(3,6),
	   Eigen::MatrixXd::Zero(3,9), I3;
} else if(pos_src==3) {
	  H << I3, Eigen::MatrixXd::Zero(3,9),
	   O3, I3, Eigen::MatrixXd::Zero(3,6),
	   Eigen::MatrixXd::Zero(3,9), I3;
}
//  H = Eigen::MatrixXd::Identity(obsStates,12);


  ROS_INFO("Initialization");
  while((start_IMU || ((start_leddar && pos_src==3) || (start_sonar && pos_src==2) || (start_MCPTAM && pos_src==0) || (start_SICK && pos_src==1))) && ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
ROS_INFO("Initialization4");
  // Init Kalmann
  pose_received=false;
  bool accel=false;
  Pkk<<2*I12;
  if(pos_src==0)
        xkk << MCPTAMpos,Eigen::MatrixXd::Zero(3,1),Eigen::MatrixXd::Zero(3,1);
  else //if(pos_src==3)
	xkk << SONARpos, angle, Eigen::MatrixXd::Zero(3,1), Eigen::MatrixXd::Zero(3,1);

  double t_last=ros::Time::now().toSec();


  while (ros::ok())
  {
    if(pose_received)
    {
      int t_now=ros::Time::now().toSec();
      double dt=t_now-t_last;
      t_last=t_now;

      // Compute the F and H matrices and z vector //
      F=KalmanU::Fmatrix(dt,6,accel);	//state matrix - kinematic relations
      Q=KalmanU::Qmatrix(dt,6,accel);	//std discrete error model matrix (Zarchan, 2005)
      //std::cout << "Fmatrix" << F << std::endl;  y: -1.79832460758

      //std::cout << "Qmatrix" << Q << std::endl;

	//Sensors inputs
      if(pos_src==0)
        z << MCPTAMpos,angle(0),angle(1),avel;
      else if(pos_src==1)
        z << SICKpos(0),SICKpos(1),SONARpos(2),SICKpos(2),angle(0),angle(1),SICKpos(2),avel;
      else if(pos_src==3)
        z << SONARpos,angle,avel;
      std::cout << "measurement" << z << std::endl;

      // Predict //
      xk1k=F*xkk;
      Pk1k=F*Pkk*F.transpose()+Q;
      //std::cout << "xk1k" << xk1k << std::endl;

      //Update //
      y     = z - H*xk1k;
      S     = H*Pk1k*H.transpose()+R;
      K     = Pk1k*H.transpose()*S.inverse();
      xk1k1 = xk1k + K*y;
      Pk1k1 = (I12 - K*H)*Pk1k;
      //std::cout << "z" << z << std::endl;
      //std::cout << "y" << y << std::endl;
      xkk = xk1k1;
      Pkk = Pk1k1;



		tf::Quaternion q;

      /////////   For Real Tryphon  ???????    ///////////////////
      /*
      if(pos_src==0)
          {transform.setOrigin( tf::Vector3(MCPTAMpos(0), MCPTAMpos(1), MCPTAMpos(2)) );
      q.setEulerZYX(MCPTAMpos(3), MCPTAMpos(4), MCPTAMpos(5));}
      else if(pos_src==3)
          {transform.setOrigin( tf::Vector3(SONARpos(0), SONARpos(1), SONARpos(2)) );
      q.setEulerZYX(MCPTAMpos(3), MCPTAMpos(4), MCPTAMpos(5));}
      else if(pos_src==1)
          {transform.setOrigin( tf::Vector3(SICKpos(0), SICKpos(1), SONARpos(2)) );
      q.setEulerZYX( SICKpos(2),angle(1), angle(0));}
      */
        

      ////////For Gazebo//////////  
      if(pos_src==0)
          {transform.setOrigin( tf::Vector3(MCPTAMpos(0), MCPTAMpos(1), MCPTAMpos(2)) );
      q.setEulerZYX(MCPTAMpos(5), MCPTAMpos(4), MCPTAMpos(3));}
      else if(pos_src==3)
          {transform.setOrigin( tf::Vector3(SONARpos(0), SONARpos(1), SONARpos(2)) );
      q.setEulerZYX(MCPTAMpos(5), MCPTAMpos(4), MCPTAMpos(3));}
      else if(pos_src==1)
          {transform.setOrigin( tf::Vector3(SICKpos(0), SICKpos(1), SONARpos(2)) );
      q.setEulerZYX( SICKpos(2),angle(1), angle(0));}
      
      transform.setRotation(q);
      sprintf(rosname,"Gumstick_%s",ip_address.c_str());
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", rosname));

      transform.setOrigin( tf::Vector3(xkk(0), xkk(1), xkk(2)) );
      tf::Quaternion q2;
      q2.setEulerZYX(xkk(5), xkk(4), xkk(3));
      transform.setRotation(q2);
      sprintf(rosname,"Gumstick_filtered_%s",ip_address.c_str());
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", rosname));



      geometry_msgs::PoseStamped PoseF;
      PoseF.header.stamp=ros::Time::now();
      PoseF.pose.position.x=xkk(0);
      PoseF.pose.position.y=xkk(1);
      PoseF.pose.position.z=xkk(2);

      PoseF.pose.orientation.x=xkk(3);
      PoseF.pose.orientation.y=xkk(4);
      PoseF.pose.orientation.z=xkk(5);


      geometry_msgs::TwistStamped TwistF;
      TwistF.header.stamp=ros::Time::now();
      TwistF.twist.linear.x=xkk(6);
      TwistF.twist.linear.y=xkk(7);
      TwistF.twist.linear.z=xkk(8);

      TwistF.twist.angular.x=xkk(9);
      TwistF.twist.angular.y=xkk(10);
      TwistF.twist.angular.z=xkk(11);

      pubP.publish(PoseF);
      pubV.publish(TwistF);




      pose_received=false;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}





/*

  yaml::loadPose(relative_pose_CM, rel_pos_CM, rel_quat_CM);
  yaml::loadPose(relative_pose_camera, rel_pos_camera, rel_quat_camera);
  yaml::loadPose(relative_pose_IMU, rel_pos_IMU, rel_quat_IMU);
  yaml::loadPose(relative_pose_SICK, rel_pos_SICK, rel_quat_SICK);

  std::string relative_pose_CM;
  if (nh.getParam("relative_pose_CM", relative_pose_CM))
  {
    ROS_INFO("Calibration CM: %s", relative_pose_CM.c_str());
  }
  else
  {
    ROS_FATAL("Failed to get the param relative_pose_CM");
    ros::shutdown();
  }

  std::string relative_pose_camera;
  if (nh.getParam("relative_pose_camera", relative_pose_camera))
  {
    ROS_INFO("Calibration camera: %s", relative_pose_camera.c_str());
  }
  else
  {
    ROS_FATAL("Failed to get the param relative_pose_camera");
    ros::shutdown();
  }

  std::string relative_pose_IMU;
  if (nh.getParam("relative_pose_IMU", relative_pose_IMU))
  {
    ROS_INFO("Calibration IMU: %s", relative_pose_IMU.c_str());
  }
  else
  {
    ROS_FATAL("Failed to get the param relative_pose_IMU");
    ros::shutdown();
  }

  std::string relative_pose_SICK;
  if (nh.getParam("relative_pose_SICK", relative_pose_SICK))
  {
    ROS_INFO("Calibration SICK: %s", relative_pose_SICK.c_str());
  }
  else
  {
    ROS_FATAL("Failed to get the param relative_pose_SICK");
    ros::shutdown();
  }*/
