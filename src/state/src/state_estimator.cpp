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
#include <sensor_msgs/Imu.h>

//libraries for the sonars and the compass
#include "sensors/sonar.h"
#include "sensors/sonarArray.h"
#include "sensors/compass.h"

// YAML file
#include <YAMLParser.h>

// YAML file
#include <Euler_utility.h>




// Global variable //
double tIMU, tCam, tSICK, tSonars, tComp;
sensor_msgs::Imu Imu;

bool start=false;

//Publishers //



//Subscribers //
void subImu(const sensor_msgs::Imu msg)
{
  Imu=msg;
  start=true;
}




int main(int argc, char **argv)
{
  ros::init(argc, argv, "state_estimator");
  ros::NodeHandle nh("~");


  // Getting the parameters //
  std::string ip;
  if (nh.getParam("ip", ip))
  {
    ROS_INFO("state_estimator started with address: %s", ip.c_str());
  }
  else
  {
    ROS_FATAL("Failed to get the ip");
    ros::shutdown();
  }

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
  }

  // Loading from YAML files //
  Eigen::Vector3d rel_pos_CM, rel_pos_camera, rel_pos_IMU, rel_pos_SICK;
  Eigen::Quaterniond rel_quat_CM, rel_quat_camera, rel_quat_IMU, rel_quat_SICK;

  yaml::loadPose(relative_pose_CM, rel_pos_CM, rel_quat_CM);
  yaml::loadPose(relative_pose_camera, rel_pos_camera, rel_quat_camera);
  yaml::loadPose(relative_pose_IMU, rel_pos_IMU, rel_quat_IMU);
  yaml::loadPose(relative_pose_SICK, rel_pos_SICK, rel_quat_SICK);


  // State //
  Eigen::Vector3d biasGyros;

  Eigen::Vector3d pos;
  Eigen::Vector3d euler;

  Eigen::Vector3d vel;
  Eigen::Vector3d angVel;


  char rosname[100];
  // Publishers //


  // SUbscribers //
  sprintf(rosname,"/%s/raw_imu",ip.c_str());
  ros::Subscriber subI = nh.subscribe(rosname, 1, subImu);

  static tf::TransformBroadcaster br;
  tf::Transform transform;



  ros::Rate loop_rate(10);

  // variables //
  bool init=true;

  double t_1, dt;

  double Roll,Pitch;

  Eigen::Quaterniond quat;



  // Matrices and vectors for the kalman filter //
  Eigen::VectorXd xK1K1Ang(9),xK1KAng(9),xKKAng(9);
  Eigen::VectorXd yAng(6), zAng(6); // measurement redidual

  Eigen::MatrixXd fAng(9,9), FAng(9,9), QAng(9,9), RAng(6,6), PAngK1K1(9,9), PAngK1K(9,9), PAngKK(9,9), SAng(6,6),  KAng(9,6), HAng(6,9), I9(9,9), I3(3,3), O3(3,3);

  I3<< 1,  0,  0,
  0,  1,  0,
  0,   0, 1;

  O3<< 0,  0,  0,
  0,  0,  0,
  0,   0, 0;

  I9 << I3,O3,O3,O3,I3,O3,O3,O3,I3;

  QAng=0.01*I9;


  RAng << 0.03*I3,O3,O3,0.3*I3;




  while (ros::ok())
  {
    if(start)
    {
      if(init)
      {
        dt=0.1;
        init=false;
        t_1=Imu.header.stamp.toSec();
        PAngKK<<2*I3,O3,O3,O3,2*I3,O3,O3,O3,2*I3;
        xKKAng << 0,0,0,0,0,0,0,0,0;
      }
      else
      {
        dt=Imu.header.stamp.toSec()-t_1;
        t_1=Imu.header.stamp.toSec();
      }

      // Compute the F, f and H matrices //
      FAng<< I3,dt*I3,O3,O3,I3,O3,O3,O3,I3;
      fAng<< I3,dt*I3,O3,O3,I3,O3,O3,O3,I3;

      HAng<< I3,O3,O3,O3,I3,I3;


      euler::getRollPitchIMU(Imu,Roll,Pitch);

      zAng[0]=Roll;
      zAng[1]=Pitch;
      zAng[2]=0;

      zAng[3]=Imu.angular_velocity.x;
      zAng[4]=Imu.angular_velocity.x;
      zAng[5]=Imu.angular_velocity.x;


      // Predict //
      xK1KAng=FAng*xKKAng;
      PAngK1K=FAng*PAngK1K1*FAng.transpose()+QAng;

      //Update //
      yAng     = zAng - HAng*xK1KAng;
      SAng     = HAng*PAngK1K*HAng.transpose()+RAng;
      KAng     = PAngK1K*HAng.transpose()*SAng.inverse();
      xK1K1Ang = xK1KAng + KAng*yAng;
      PAngK1K1 = (I9 - KAng*HAng)*PAngK1K;

      // Update values //

      xKKAng = xK1K1Ang;
      PAngKK = PAngK1K1;

      ROS_INFO("Roll: %f, Pitch %f, Roll: %f, Pitch %f",Pitch, Roll, xK1K1Ang[1], xK1K1Ang[0]);

      double Roll2=xKKAng(0);
      double Pitch2=xKKAng(1);


      transform.setOrigin( tf::Vector3(0, 0, 0.0) );
      tf::Quaternion q;
      q.setEulerZYX(0, Pitch, Roll);
      transform.setRotation(q);
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "Gumstick"));

      transform.setOrigin( tf::Vector3(0, 0.5, 0.0) );
      tf::Quaternion q2;
      q2.setEulerZYX(0, Pitch2, Roll2);
      transform.setRotation(q2);
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "Gumstick_filtered"));




    }




    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
