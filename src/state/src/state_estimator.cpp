//library for ros
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>


// Standard C/C++ libraries
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <ctime>
#include <iostream>

// Eigen linraries
#include <Eigen/Geometry>
#include <Eigen/Dense>


// ROS messqges libraries
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include <sensor_msgs/Imu.h>

//libraries for the sonars and the compass
#include "sensors/sonar.h"
#include "sensors/sonarArray.h"
#include "sensors/compass.h"

// YAML file
//#include <YAMLParser.h>

// Euler file
#include <Euler_utility.h>

// Kalman file
#include <Kalman_utility.h>




// Global variable //
double tIMU, tMCPTAM, tSICK, tSonars, tComp;
sensor_msgs::Imu Imu;

bool start_IMU=true;
bool start_MCPTAM=true;
bool pose_received;

Eigen::Vector3d pos, angle, avel;
Eigen::Vector2d rollPitchI;

Eigen::Vector3d CMIMUpos(1,0,-1.125); // vector postion from Center of mass to IMU in tryphon frame
Eigen::Matrix3d Rmatrix, CPCMIMUmatrix, RIMUmatrix,CPCMCmatrix;
Eigen::Quaterniond quatIMU(0.99255, 0,0.12187, 0); // quat of the rotation matrix between the tryphon frame and the IMU frame
Eigen::Quaterniond quatMCPTAM(1, 0,0, 0); // quat of the rotation matrix between the tryphon frame and the IMU frame



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
void subImu(const sensor_msgs::Imu Imu_msg)
{
  
  double roll, pitch;
  EulerU::getRollPitchIMU(Imu_msg,roll,pitch);
  rollPitchI(0)=(rollPitchI(0)+roll)/2; // low pass filter
  rollPitchI(1)=(rollPitchI(1)+0.244+pitch)/2; // low pass filter

  Eigen::Vector3d avel_temp;
  avel_temp(0)=Imu_msg.angular_velocity.x; // defined in IMU frame
  avel_temp(1)=Imu_msg.angular_velocity.y;
  avel_temp(2)=Imu_msg.angular_velocity.z;
  avel_temp=RIMUmatrix*avel_temp;  // defined in body frame

  avel_temp=EulerU::RbodyEuler(rollPitchI(0),rollPitchI(1))*avel_temp;
  avel=(avel+avel_temp)/2;
    if(start_IMU)
  {
    start_IMU=false;
  }

}

void subMCPTAM(const geometry_msgs::PoseArray Aposes) // with MCPTAM
{
  tMCPTAM=ros::Time::now().toSec();
  pose_received=true;
  geometry_msgs::Pose Pose=Aposes.poses[0];
  

  pos(0)=Pose.position.x; // defined in global frame
  pos(1)=Pose.position.y;
  pos(2)=Pose.position.z;
  Eigen::Quaterniond quat(Pose.orientation.w,Pose.orientation.x,Pose.orientation.y,Pose.orientation.z);
  Eigen::Quaterniond quat1(Pose.orientation.w,Pose.orientation.x,Pose.orientation.y,Pose.orientation.z);
  Eigen::Quaterniond quat2(Pose.orientation.w,Pose.orientation.x,Pose.orientation.y,Pose.orientation.z);
  Eigen::Quaterniond quat3(Pose.orientation.w,Pose.orientation.x,Pose.orientation.y,Pose.orientation.z);
  quat=quat*quatMCPTAM.inverse();  // compute the quaternion between the vision world and the tryphon frame

  angle(0)=atan2(2*(quat.w()*quat.x()+quat.y()*quat.z()),1-2*(quat.x()*quat.x()+quat.y()*quat.y()));
  angle(1)=asin(2*(quat.w()*quat.y()-quat.z()*quat.x()));
  angle(2)=atan2(2*(quat.w()*quat.z()+quat.x()*quat.y()),1-2*(quat.z()*quat.z()+quat.y()*quat.y()));
  Rmatrix=quat.toRotationMatrix();

  pos=pos-Rmatrix*CMIMUpos;  // offset due to the fact that the pose is the one of the IMU
  if(start_MCPTAM)
  {
    start_MCPTAM=false;
  }
}




int main(int argc, char **argv)
{
  ros::init(argc, argv, "state_estimator");
  ros::NodeHandle nh;


  // Loading from YAML files //
  Eigen::Vector3d rel_pos_CM, rel_pos_camera, rel_pos_IMU, rel_pos_SICK;
  Eigen::Quaterniond rel_quat_CM, rel_quat_camera, rel_quat_IMU, rel_quat_SICK;


  char rosname[100];
  // Publishers //


  // SUbscribers //
  ros::Subscriber subI = nh.subscribe("raw_imu", 1, subImu);
  ros::Subscriber subM = nh.subscribe("mcptam/tracker_pose_array",1,subMCPTAM);

  ros::Publisher pubP = nh.advertise<geometry_msgs::Pose>("state_estimator/pose",1);
  ros::Publisher pubV = nh.advertise<geometry_msgs::Twist>("state_estimator/vel",1);

  static tf::TransformBroadcaster br;
  tf::Transform transform;



  ros::Rate loop_rate(100);

  // variables //


  // Relatives frames variables
  RIMUmatrix=quatIMU.toRotationMatrix(); // need to be computed only once
  CPCMIMUmatrix=CPM(CMIMUpos);

  Eigen::Quaterniond quat;



  // Matrices and vectors for the kalman filter //
  Eigen::VectorXd xk1k1(12),xk1k(12),xkk(12);
  Eigen::VectorXd z(11), y(11); // measurement and measurement residual
  Eigen::VectorXd yMCPTAM(6); // MCPTAM measurement
  Eigen::VectorXd yIMU(5); // IMU measurement

  Eigen::MatrixXd F(12,12), Q(12,12), Pk1k1(12,12), Pk1k(12,12), Pkk(12,12), I6(6,6), O6(3,3), I3(3,3), O3(3,3), I13(6,6), I12(6,6), RIMU(5,5), RMCPTAM(6,6),R(11,11), S(11,11), K(12,11), H(11,12);
  //Eigen::MatrixXd R(6,6), S(6,6), K(12,6), H(6,12);

  I3=Eigen::MatrixXd::Identity(3,3);
  O3=Eigen::MatrixXd::Zero(3,3);

  I6=Eigen::MatrixXd::Identity(6,6);
  O6=Eigen::MatrixXd::Zero(6,6);

  I12=Eigen::MatrixXd::Identity(12,12);
  RMCPTAM << 0.1*I3,O3,O3,0.05*I3;
  ROS_INFO("Initialization test");
  RIMU << 0.05*Eigen::MatrixXd::Identity(2,2),Eigen::MatrixXd::Zero(2,3),Eigen::MatrixXd::Zero(3,2),0.05*I3;


  R << RMCPTAM,Eigen::MatrixXd::Zero(6,5),Eigen::MatrixXd::Zero(5,6),RIMU;
  //R << RMCPTAM;

  H<< I6,Eigen::MatrixXd::Zero(6,6),
          Eigen::MatrixXd::Zero(2,3),Eigen::MatrixXd::Identity(2,2),Eigen::MatrixXd::Zero(2,7),
          Eigen::MatrixXd::Zero(3,9),Eigen::MatrixXd::Identity(3,3);
  //H<< I6,Eigen::MatrixXd::Zero(6,6);


  ROS_INFO("Initialization");
  while((start_IMU || start_MCPTAM) && ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  // Init Kalmann
  pose_received=false;
  bool accel=false;


  Pkk<<2*Eigen::MatrixXd::Identity(12,12);
  xkk << pos,angle,Eigen::MatrixXd::Zero(3,1),Eigen::MatrixXd::Zero(3,1);
  double t_1=ros::Time::now().toSec();


  while (ros::ok())
  {
    if(pose_received)
    {
      double dt=tMCPTAM-t_1;
      t_1=ros::Time::now().toSec();

      // Compute the F and H matrices and z vector //
      F=KalmanU::Fmatrix(dt,6,accel);
      Q=KalmanU::Qmatrix(dt,6,accel);
      //std::cout << "Fmatrix" << F << std::endl;
      //std::cout << "Qmatrix" << Q << std::endl;



      z << pos,angle,rollPitchI,avel;
      //std::cout << "measurement" << z << std::endl;

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

      // Update values //

      xkk = xk1k1;
      Pkk = Pk1k1;


      transform.setOrigin( tf::Vector3(pos(0), pos(1), pos(2)) );
      tf::Quaternion q;
      q.setEulerZYX(angle(0), angle(1), angle(2));
      transform.setRotation(q);
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "Gumstick"));

      transform.setOrigin( tf::Vector3(xkk(0), xkk(1), xkk(2)) );
      tf::Quaternion q2;
      q2.setEulerZYX(xkk(3), xkk(4), xkk(5));
      transform.setRotation(q2);
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "Gumstick_filtered"));

      geometry_msgs::Pose PoseF;
      PoseF.position.x=xkk(0);
      PoseF.position.y=xkk(1);
      PoseF.position.z=xkk(2);

      PoseF.orientation.x=xkk(3);
      PoseF.orientation.y=xkk(4);
      PoseF.orientation.z=xkk(5);


      geometry_msgs::Twist TwistF;
      TwistF.linear.x=xkk(6);
      TwistF.linear.y=xkk(7);
      TwistF.linear.z=xkk(8);

      TwistF.angular.x=xkk(9);
      TwistF.angular.y=xkk(10);
      TwistF.angular.z=xkk(11);

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
