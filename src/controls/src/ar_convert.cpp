
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/WrenchStamped.h"
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <Euler_utility.h>
#include "vects2geoMsgs.cpp"
#include <ros/ros.h>

Eigen::Vector3d ALVAR1pos,ALVAR2pos;
 Eigen::Vector3d ALVAR1angle,ALVAR2angle;
Eigen::Matrix3d Rmatrix1,Rmatrix2;
geometry_msgs::PoseStamped alvarp1, alvarp2;
geometry_msgs::TwistStamped alvarv1, alvarv2, alvarv1_old, alvarv2_old;


void subALVAR1(const ar_track_alvar_msgs::AlvarMarkers Aposes1)
{
  ar_track_alvar_msgs::AlvarMarker alvartemp;
  alvartemp=Aposes1.markers[0];
  geometry_msgs::Pose Pose=alvartemp.pose.pose ; //Pose=Aposes.pose;
  Eigen::Vector3d ALVAR1position,ALVAR2position, temp;

  ALVAR1position(0)=Pose.position.x; // defined in global frame
  ALVAR1position(1)=Pose.position.y;
  ALVAR1position(2)=Pose.position.z;
  Eigen::Quaterniond quat(Pose.orientation.w,Pose.orientation.x,Pose.orientation.y,Pose.orientation.z);
  


  

Eigen::Quaterniond quattemp=quat.inverse();
Rmatrix1=quat.inverse().toRotationMatrix();
Rmatrix2=quattemp.toRotationMatrix();

//temp=ALVAR1position;
ALVAR1position=Rmatrix1*ALVAR1position;

//ALVAR2position=Rmatrix2*temp;



  ALVAR1angle(0)=atan2(2*(quat.w()*quat.x()+quat.y()*quat.z()),1-2*(quat.x()*quat.x()+quat.y()*quat.y()));
  ALVAR1angle(1)=asin(2*(quat.w()*quat.y()-quat.z()*quat.x()));
  ALVAR1angle(2)=atan2(2*(quat.w()*quat.z()+quat.x()*quat.y()),1-2*(quat.z()*quat.z()+quat.y()*quat.y()));
 
  ALVAR1pos(0)=ALVAR1position(0);
  ALVAR1pos(1)=ALVAR1position(1);
  ALVAR1pos(2)=ALVAR1position(2);

 // ROS_INFO("x: %f,%f,y:%f,%f,z:%f,%f", ALVAR1position(0),ALVAR2position(0), ALVAR1position(1),ALVAR2position(1), ALVAR1position(2),ALVAR2position(2) );
}

int main(int argc, char **argv)
{



ros::init(argc, argv, "ar_convert");
ros::NodeHandle nh;

ros::Subscriber  pose1_sub= nh.subscribe("/192_168_10_243/artags1/artag1/ar_pose_marker",1,subALVAR1);

ros::Publisher  ar_pub = nh.advertise<geometry_msgs::PoseStamped>("/192_168_10_243/pose_ar_frame",1); //to work currently with FD

ros::Rate loop_rate(10);

while (ros::ok())
 {

ros::spinOnce();





alvarp1.header.stamp=ros::Time::now();
alvarp1.pose=vects2pose(ALVAR1pos,ALVAR1angle);
  	ar_pub.publish(alvarp1);
}

return 0;
}

