
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/WrenchStamped.h"

#include <Euler_utility.h>
#include "vects2geoMsgs.cpp"


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
  Eigen::Vector3d ALVAR1position;
   Eigen::Quaterniond quatpos(.7071,-.7071,0,0);


  ALVAR1position(0)=Pose.position.x; // defined in global frame
  ALVAR1position(1)=Pose.position.y;
  ALVAR1position(2)=Pose.position.z;
  Eigen::Quaterniond quat(Pose.orientation.w,Pose.orientation.x,Pose.orientation.y,Pose.orientation.z);
  


  
quat=quatpos*quat;

Rmatrix1=quat.toRotationMatrix();
ALVAR1position=Rmatrix1*ALVAR1position;

  ALVAR1angle(0)=atan2(2*(quat.w()*quat.x()+quat.y()*quat.z()),1-2*(quat.x()*quat.x()+quat.y()*quat.y()));
  ALVAR1angle(1)=asin(2*(quat.w()*quat.y()-quat.z()*quat.x()));
  ALVAR1angle(2)=atan2(2*(quat.w()*quat.z()+quat.x()*quat.y()),1-2*(quat.z()*quat.z()+quat.y()*quat.y()));
 
  ALVAR1pos(0)=ALVAR1position(0);
  ALVAR1pos(1)=ALVAR1position(1);
  ALVAR1pos(2)=ALVAR1position(2);
}

int main(int argc, char **argv)
{



ros::init(argc, argv, "ar_convert");
ros::NodeHandle nh;

ros::Subscriber  pose1_sub= nh.subscribe("/192_168_10_243/ar_pose",1,subALVAR1);

ros::Publisher  ar_pub = nh.advertise<geometry_msgs::Wrench>("/192_168_10_243/pose_ar_frame",1); //to work currently with FD


while (ros::ok())
 {

ros::spinOnce();





alvarp1.header.stamp=ros::Time::now();
alvarp1.pose=vects2pose(ALVAR1pos,ALVAR1angle);
  	alvarp1_pub.publish(alvarp1);
}

ros::Rate loop_rate(10);
