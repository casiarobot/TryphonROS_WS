



#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>
// Euler file
#include <Euler_utility.h>
#include "vects2geoMsgs.cpp"
#include <math.h>

 Eigen::Vector3d ALVAR1pos,ALVAR2pos;
 Eigen::Vector3d ALVAR1angle,ALVAR2angle;
Eigen::Matrix3d Rmatrix1,Rmatrix2;
geometry_msgs::PoseStamped alvarp1, alvarp2;
geometry_msgs::TwistStamped alvarv1, alvarv2, alvarv1_old, alvarv2_old;
double pos1[6][4]; //used for finite difference
double pos2[6][4]; //used for finite difference
double h=0.1;



void noise1()
{
	if(alvarv1.twist.linear.x<0.003 && alvarv1.twist.linear.x>-0.003)
	{alvarv1.twist.linear.x=0;}
	if(alvarv1.twist.linear.y<0.003 && alvarv1.twist.linear.y>-0.003)
	{alvarv1.twist.linear.y=0;}
	if(alvarv1.twist.linear.z<0.003 && alvarv1.twist.linear.z>-0.003)
	{alvarv1.twist.linear.z=0;}
	if(fabs(alvarv1.twist.angular.x<0.003))
	{alvarv1.twist.angular.x=0;}
	if(fabs(alvarv1.twist.angular.y<0.003))
	{alvarv1.twist.angular.y=0;}
	if(fabs(alvarv1.twist.angular.z<0.003))
	{alvarv1.twist.angular.z=0;}


}

void noise2()
{
	if(alvarv2.twist.linear.x<0.003 && alvarv2.twist.linear.x>-0.003)
	{alvarv2.twist.linear.x=0;}
	if(alvarv2.twist.linear.y<0.003 && alvarv2.twist.linear.y>-0.003)
	{alvarv2.twist.linear.y=0;}
	if(alvarv2.twist.linear.z<0.003 && alvarv2.twist.linear.z>-0.003)
	{alvarv2.twist.linear.z=0;}
	if(fabs(alvarv2.twist.angular.x<0.003))
	{alvarv2.twist.angular.x=0;}
	if(fabs(alvarv2.twist.angular.y<0.003))
	{alvarv2.twist.angular.y=0;}
	if(fabs(alvarv2.twist.angular.z<0.003))
	{alvarv2.twist.angular.z=0;}

}

void subALVAR1(const ar_track_alvar_msgs::AlvarMarkers Aposes1)
{
  ar_track_alvar_msgs::AlvarMarker alvartemp;
  alvartemp=Aposes1.markers[0];
  geometry_msgs::Pose Pose=alvartemp.pose.pose ; //Pose=Aposes.pose;
  Eigen::Vector3d ALVAR1position;

  ALVAR1position(0)=Pose.position.x; // defined in global frame
  ALVAR1position(1)=Pose.position.y;
  ALVAR1position(2)=Pose.position.z;
  Eigen::Quaterniond quat(Pose.orientation.w,Pose.orientation.x,Pose.orientation.y,Pose.orientation.z);
  
//ROS_INFO("Pose.z=%f",Pose.position.z);

//Eigen::Quaterniond quatpos(0,0,.7071,-.7071); //to make x y z of camera/artag output aloligned with tryphon body frame
Eigen::Quaterniond quatpos(.7071,-.7071,0,0);
Eigen::Quaterniond quatang(0,1,0,0); //to make orientation all 0 when docking face alligned
quat=quat*quatang.inverse();  


Rmatrix1=quatpos.toRotationMatrix();
ALVAR1position=Rmatrix1*ALVAR1position;

  ALVAR1angle(0)=atan2(2*(quat.w()*quat.x()+quat.y()*quat.z()),1-2*(quat.x()*quat.x()+quat.y()*quat.y()));
  ALVAR1angle(1)=asin(2*(quat.w()*quat.y()-quat.z()*quat.x()));
  ALVAR1angle(2)=atan2(2*(quat.w()*quat.z()+quat.x()*quat.y()),1-2*(quat.z()*quat.z()+quat.y()*quat.y()));
 
  ALVAR1pos(0)=ALVAR1position(0);
  ALVAR1pos(1)=ALVAR1position(1);
  ALVAR1pos(2)=ALVAR1position(2);

for (int i=0; i<6 ; i++)
{
	for (int j=0; j<3 ; j++)
	{
		pos1[i][j]=pos1[i][j+1];
	}
}

		
		
		pos1[0][3]=ALVAR2pos(0);
		alvarv1.twist.linear.x=(1.83333*pos1[0][3]-3*pos1[0][2]+1.5*pos1[0][1]-0.33333*pos1[0][0])/h;
		if(alvarv1.twist.linear.x-alvarv1_old.twist.linear.x>0.07 || alvarv1.twist.linear.x-alvarv1_old.twist.linear.x<-0.07){alvarv1.twist.linear.x=alvarv1_old.twist.linear.x;}
		ROS_INFO("pos1 x:%f",alvarv1.twist.linear.x);
		pos1[1][3]=ALVAR2pos(1);
		alvarv1.twist.linear.y=(1.83333*pos1[1][3]-3*pos1[1][2]+1.5*pos1[1][1]-0.33333*pos1[1][0])/h;
		if(alvarv1.twist.linear.y-alvarv1_old.twist.linear.y>0.07 || alvarv1.twist.linear.y-alvarv1_old.twist.linear.y<-0.07){alvarv1.twist.linear.y=alvarv1_old.twist.linear.y;}
		pos1[2][3]=ALVAR2pos(2);
		alvarv1.twist.linear.z=(1.83333*pos1[2][3]-3*pos1[2][2]+1.5*pos1[2][1]-0.33333*pos1[2][0])/h;
		if(alvarv1.twist.linear.z-alvarv1_old.twist.linear.z>0.07 || alvarv1.twist.linear.z-alvarv1_old.twist.linear.z<-0.07){alvarv1.twist.linear.z=alvarv1_old.twist.linear.z;}
		pos1[3][3]=ALVAR2angle(0);
		alvarv1.twist.angular.x=(1.83333*pos1[3][3]-3*pos1[3][2]+1.5*pos1[3][1]-0.33333*pos1[3][0])/h;
		if(fabs(alvarv1.twist.angular.x-alvarv1_old.twist.angular.x)>0.03){alvarv1.twist.angular.x=alvarv1_old.twist.angular.x;}
		pos1[4][3]=ALVAR2angle(1);
		alvarv1.twist.angular.y=(1.83333*pos1[4][3]-3*pos1[4][2]+1.5*pos1[4][1]-0.33333*pos1[4][0])/h;
		if(fabs(alvarv1.twist.angular.y-alvarv1_old.twist.angular.y)>0.03){alvarv1.twist.angular.y=alvarv1_old.twist.angular.y;}
		pos1[5][3]=ALVAR2angle(2);
		alvarv1.twist.angular.z=(1.83333*pos1[5][3]-3*pos1[5][2]+1.5*pos1[5][1]-0.33333*pos1[5][0])/h;
		if(fabs(alvarv1.twist.angular.z-alvarv1_old.twist.angular.z)>0.03){alvarv1.twist.angular.z=alvarv1_old.twist.angular.z;}

alvarv1_old=alvarv1;


}


void subALVAR2(const ar_track_alvar_msgs::AlvarMarkers Aposes2) //camera frame on other tryphon
{
  ar_track_alvar_msgs::AlvarMarker alvartemp;
  alvartemp=Aposes2.markers[0];
  geometry_msgs::Pose Pose=alvartemp.pose.pose ; //Pose=Aposes.pose;
  Eigen::Vector3d ALVAR2position;

  ALVAR2position(0)=-Pose.position.x; //  switch direction of vector
  ALVAR2position(1)=-Pose.position.y;
  ALVAR2position(2)=-Pose.position.z;
  Eigen::Quaterniond quat(Pose.orientation.w,Pose.orientation.x,Pose.orientation.y,Pose.orientation.z);
  
Eigen::Quaterniond quatpos(0, 0, .7071, .7071); //to make x y z of camera/artag output alligned with tryphon body frame
Eigen::Quaterniond quatang(0,1,0,0); //to make orientation all 0 when docking face alligned
Eigen::Quaterniond quattemp;

quattemp=quatpos*quat.inverse();
Rmatrix2=quattemp.toRotationMatrix();
ALVAR2position=Rmatrix2*ALVAR2position;

quat=quat*quatang.inverse(); //allign offset when facin each other 

  ALVAR2angle(0)=atan2(2*(quat.w()*quat.x()+quat.y()*quat.z()),1-2*(quat.x()*quat.x()+quat.y()*quat.y()));
  ALVAR2angle(1)=asin(2*(quat.w()*quat.y()-quat.z()*quat.x()));
  ALVAR2angle(2)=atan2(2*(quat.w()*quat.z()+quat.x()*quat.y()),1-2*(quat.z()*quat.z()+quat.y()*quat.y()));
  //Rmatrix2=quat.toRotationMatrix();
  ALVAR2pos(0)=ALVAR2position(0);
  ALVAR2pos(1)=ALVAR2position(1);
  ALVAR2pos(2)=ALVAR2position(2);

for (int i=0; i<6 ; i++)
{
	for (int j=0; j<3 ; j++)
	{
		pos2[i][j]=pos2[i][j+1];
	}
}

		pos2[0][3]=ALVAR2pos(0);
		alvarv2.twist.linear.x=(1.83333*pos2[0][3]-3*pos2[0][2]+1.5*pos2[0][1]-0.33333*pos2[0][0])/h;
		if(alvarv2.twist.linear.x-alvarv2_old.twist.linear.x>0.07 || alvarv2.twist.linear.x-alvarv2_old.twist.linear.x<-0.07){alvarv2.twist.linear.x=alvarv2_old.twist.linear.x;}
		pos2[1][3]=ALVAR2pos(1);
		alvarv2.twist.linear.y=(1.83333*pos2[1][3]-3*pos2[1][2]+1.5*pos2[1][1]-0.33333*pos2[1][0])/h;
		if(alvarv2.twist.linear.y-alvarv2_old.twist.linear.y>0.07 || alvarv2.twist.linear.y-alvarv2_old.twist.linear.y<-0.07 ){alvarv2.twist.linear.y=alvarv2_old.twist.linear.y;}
		pos2[2][3]=ALVAR2pos(2);
		alvarv2.twist.linear.z=(1.83333*pos2[2][3]-3*pos2[2][2]+1.5*pos2[2][1]-0.33333*pos2[2][0])/h;
		if(alvarv2.twist.linear.z-alvarv2_old.twist.linear.z>0.07 || alvarv2.twist.linear.z-alvarv2_old.twist.linear.z<-0.07){alvarv2.twist.linear.z=alvarv2_old.twist.linear.z;}
		pos2[3][3]=ALVAR2angle(0);
		alvarv2.twist.angular.x=(1.83333*pos2[3][3]-3*pos2[3][2]+1.5*pos2[3][1]-0.33333*pos2[3][0])/h;
		if(fabs(alvarv2.twist.angular.x-alvarv2_old.twist.angular.x)>0.03){alvarv2.twist.angular.x=alvarv2_old.twist.angular.x;}
		pos2[4][3]=ALVAR2angle(1);
		alvarv2.twist.angular.y=(1.83333*pos2[4][3]-3*pos2[4][2]+1.5*pos2[4][1]-0.33333*pos2[4][0])/h;
		if(fabs(alvarv2.twist.angular.y-alvarv2_old.twist.angular.y)>0.03){alvarv2.twist.angular.y=alvarv2_old.twist.angular.y;}
		pos2[5][3]=ALVAR2angle(2);
		alvarv2.twist.angular.z=(1.83333*pos2[5][3]-3*pos2[5][2]+1.5*pos2[5][1]-0.33333*pos2[5][0])/h;
		if(fabs(alvarv2.twist.angular.z-alvarv2_old.twist.angular.z)>0.03){alvarv2.twist.angular.z=alvarv2_old.twist.angular.z;}


alvarv2_old=alvarv2;
}



void pose_zero(geometry_msgs::PoseStamped &p) //set any pose msg to zero
{
	p.pose.position.x=0;
	p.pose.position.y=0;
	p.pose.position.z=0;
	p.pose.orientation.w=0; 
	p.pose.orientation.x=0;
	p.pose.orientation.y=0;
	p.pose.orientation.z=0;
}

void twist_zero(geometry_msgs::TwistStamped &tw) //set any pose msg to zero
{
	tw.twist.linear.x=0;
	tw.twist.linear.y=0;
	tw.twist.linear.z=0;
	tw.twist.angular.x=0;
	tw.twist.angular.y=0;
	tw.twist.angular.z=0;
}





int main(int argc, char **argv)
{


  ros::init(argc, argv, "ar_state");
  ros::NodeHandle nh;

ros::Subscriber sub1 = nh.subscribe("/192_168_10_243/artags/artag1/ar_pose_marker",1,subALVAR1);
ros::Subscriber sub2 = nh.subscribe("/192_168_10_244/artags/artag1/ar_pose_marker",1,subALVAR2);

ros::Publisher  alvarp1_pub = nh.advertise<geometry_msgs::PoseStamped>("/192_168_10_243/ar_pose",1);
ros::Publisher  alvarp2_pub = nh.advertise<geometry_msgs::PoseStamped>("/192_168_10_244/ar_pose",1);
ros::Publisher  alvarv1_pub = nh.advertise<geometry_msgs::TwistStamped>("/192_168_10_243/ar_vel",1);
ros::Publisher  alvarv2_pub = nh.advertise<geometry_msgs::TwistStamped>("/192_168_10_244/ar_vel",1);

ros::Rate loop_rate(10);

pose_zero(alvarp1);
pose_zero(alvarp2);
twist_zero(alvarv1);
twist_zero(alvarv2);
twist_zero(alvarv1_old);
twist_zero(alvarv2_old);

for (int i=0; i<6 ; i++)
{
	for (int j=0; j<6 ; j++)
	{
		pos1[i][j]=0;
		pos2[i][j]=0;
	}

}

while (ros::ok())
 {

ros::spinOnce();





alvarp1.header.stamp=ros::Time::now();
alvarp2.header.stamp=ros::Time::now();
alvarp1.pose=vects2pose(ALVAR1pos,ALVAR1angle);
alvarp2.pose=vects2pose(ALVAR2pos,ALVAR2angle);


//alvarv1=update_pos_and_give_vel1(alvarp1);
//alvarv2=update_pos_and_give_vel2(alvarp2); //header stamp done above

//noise1();
//noise2();

alvarv1.header.stamp=ros::Time::now();
alvarv2.header.stamp=ros::Time::now();

  	alvarp1_pub.publish(alvarp1);
  	alvarp2_pub.publish(alvarp2);
  	alvarv1_pub.publish(alvarv1);
  	alvarv2_pub.publish(alvarv2);

}

	return 0;
}

