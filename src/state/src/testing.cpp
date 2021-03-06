


/*
#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>
// Euler file
#include <Euler_utility.h>
#include "vects2geoMsgs.cpp"
#include <math.h>

 Eigen::Vector3d ALVAR1pos,ALVAR2pos,ALVAR1position1, ALVAR1position2,ALVAR2position1, ALVAR2position2;
 Eigen::Vector3d ALVAR1angle,ALVAR2angle,ALVARangle;
Eigen::Matrix3d Rmatrix1,Rmatrix2;
Eigen::Quaterniond quatdiff(1,0,0,0);
geometry_msgs::PoseStamped alvarp1, alvarp2;
geometry_msgs::TwistStamped alvarv1, alvarv2, alvarv1_old, alvarv2_old;
double pos[2][6][11]; //used for finite difference
double h=0.1;
double posx[10];
double meanx=.45;



//assumed both cameras on tryphon
void subALVAR1(const geometry_msgs::PoseStamped Aposes1)
{
  
  geometry_msgs::Pose Pose1=Aposes1.pose ; 


  ALVAR1position1(0)=Pose1.position.x; // defined in global frame
  ALVAR1position1(1)=Pose1.position.y;
  ALVAR1position1(2)=Pose1.position.z;



  ALVAR1pos(0)=ALVAR1position1(0);
  ALVAR1pos(1)=ALVAR1position1(1);
  ALVAR1pos(2)=ALVAR1position1(2);

//ROS_INFO("2");
}
	



void subALVAR2(const geometry_msgs::PoseStamped Aposes2)
{
  
  geometry_msgs::Pose Pose1=Aposes2.pose ; 




  ALVAR2position1(0)=Pose1.position.x; //  switch direction of vector
  ALVAR2position1(1)=Pose1.position.y;
  ALVAR2position1(2)=Pose1.position.z;
 

  ALVAR2pos(0)=ALVAR2position1(0);
  ALVAR2pos(1)=ALVAR2position1(1);
  ALVAR2pos(2)=ALVAR2position1(2);


}

void velocity_solver()
{
int i,j,k;
double mean[2][6];
double tempsumnum[2][6];
double tempsumden[2][6];
double tempsum[2][6];
/////Compute mean values

//ROS_INFO("1");

for (int i=0; i<2 ; i++)
{
	for (int j=0; j<6 ; j++)
	{
		for(k=0;k<10;k++)
		{
		pos[i][j][k]=pos[i][j][k+1];
		}
	}
}


for (i=0;i<2;i++)
{
	for(j=0;j<6;j++)
	{
	mean[i][j]=0;
		for(k=0;k<10;k++)
		{
		mean[i][j]=mean[i][j]+pos[i][j][k];
		}
	mean[i][j]=mean[i][j]/10.0;
	}
}
////

///compute velocity
for (i=0;i<2;i++)
{
	for(j=0;j<6;j++)
	{
		tempsumnum[i][j]=0.0;
		tempsumden[i][j]=0.0;
		tempsum[i][j]=0.0;
		for(k=0;k<10;k++)
		{
		tempsumnum[i][j]=tempsumnum[i][j]+((pos[i][j][k]-mean[i][j])*(posx[k]-meanx));
		tempsumden[i][j]=tempsumden[i][j]+pow(posx[k]-meanx,2);
		}
		tempsum[i][j]=tempsumnum[i][j]/tempsumden[i][j];
	}
}
////
		
	alvarv1.twist.linear.x=tempsum[0][0];
	alvarv1.twist.linear.y=tempsum[0][1];
	alvarv1.twist.linear.z=tempsum[0][2];
	alvarv1.twist.angular.x=tempsum[0][3];
	alvarv1.twist.angular.y=tempsum[0][4];
	alvarv1.twist.angular.z	=tempsum[0][5];
	
	alvarv2.twist.linear.x=tempsum[1][0];
	alvarv2.twist.linear.y=tempsum[1][1];
	alvarv2.twist.linear.z=tempsum[1][2];
	alvarv2.twist.angular.x=tempsum[1][3];
	alvarv2.twist.angular.y=tempsum[1][4];
	alvarv2.twist.angular.z	=tempsum[1][5];

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

ros::Subscriber sub1 = nh.subscribe("/192_168_10_243/ar_pose",1,subALVAR1);
ros::Subscriber sub2 = nh.subscribe("/192_168_10_244/ar_pose",1,subALVAR2);

ros::Publisher  alvarp1_pub = nh.advertise<geometry_msgs::PoseStamped>("/192_168_10_243/ar_pose1",1);
ros::Publisher  alvarp2_pub = nh.advertise<geometry_msgs::PoseStamped>("/192_168_10_243/ar_pose2",1);
ros::Publisher  alvarv1_pub = nh.advertise<geometry_msgs::TwistStamped>("/192_168_10_243/ar_vel1",1);
ros::Publisher  alvarv2_pub = nh.advertise<geometry_msgs::TwistStamped>("/192_168_10_243/ar_vel2",1);

ros::Rate loop_rate(10);

pose_zero(alvarp1);
pose_zero(alvarp2);
twist_zero(alvarv1);
twist_zero(alvarv2);

posx[0]=0.0; //set up
for (int a=0; a<9; a++)
{
posx[a+1]=posx[a]+0.1;
}

for (int i=0; i<6 ; i++)
{
	for (int j=0; j<10 ; j++)
	{
		pos[0][i][j]=0;
		pos[1][i][j]=0;
		
	}

}

while (ros::ok())
 {

ros::spinOnce();


////////////////////////////////////////////////////////////////

//////compute velocity////////


	pos[0][0][10]=ALVAR1pos(0);
	pos[0][1][10]=ALVAR1pos(1);
	pos[0][2][10]=ALVAR1pos(2);
	pos[0][3][10]=ALVAR1angle(0);
	pos[0][4][10]=ALVAR1angle(1);
	pos[0][5][10]=ALVAR1angle(2);	

	pos[1][0][10]=ALVAR2pos(0);
	pos[1][1][10]=ALVAR2pos(1);
	pos[1][2][10]=ALVAR2pos(2);
	pos[1][3][10]=ALVAR2angle(0);
	pos[1][4][10]=ALVAR2angle(1);
	pos[1][5][10]=ALVAR2angle(2);	


velocity_solver();


//remove angle difference when facing each other
ALVARangle=quatdiff.toRotationMatrix()*ALVARangle;



alvarp1.header.stamp=ros::Time::now();
alvarp2.header.stamp=ros::Time::now();
alvarp1.pose=vects2pose(ALVAR1pos,ALVARangle);
alvarp2.pose=vects2pose(ALVAR2pos,ALVARangle);

alvarv1.header.stamp=ros::Time::now();
alvarv2.header.stamp=ros::Time::now();

  	alvarp1_pub.publish(alvarp1);
  	alvarp2_pub.publish(alvarp2);
  	alvarv1_pub.publish(alvarv1);
  	alvarv2_pub.publish(alvarv2);

loop_rate.sleep();
}

	return 0;
}

*/








#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>
// Euler file
#include <Euler_utility.h>
#include "vects2geoMsgs.cpp"
#include <math.h>

 Eigen::Vector3d ALVAR1pos,ALVAR2pos,ALVAR1position1, ALVAR1position2,ALVAR2position1, ALVAR2position2;
 Eigen::Vector3d ALVAR1angle,ALVAR2angle,ALVARangle;
Eigen::Matrix3d Rmatrix1,Rmatrix2,Rmatrix1b,Rmatrix2b;
Eigen::Quaterniond quatdiff(1,0,0,0);
geometry_msgs::PoseStamped alvarp1, alvarp2;
geometry_msgs::TwistStamped alvarv1, alvarv2;
double pos[2][6][11]; //used for finite difference
double h=0.1;
double posx[10];
double meanx=.45;



//assumed both cameras on tryphon
void subALVAR1a(const geometry_msgs::PoseStamped Aposes1) //camera frame on other tryphon
{
  
 
  geometry_msgs::Pose Pose1=Aposes1.pose ;
  //Pose=Aposes.pose;


  ALVAR1position1(0)=Pose1.position.x; //  switch direction of vector
  ALVAR1position1(1)=Pose1.position.y;
  ALVAR1position1(2)=Pose1.position.z;
 
  
/////For Tryphon////////////
//Eigen::Quaterniond quatpos(.7071,-.7071,0,0); //to make x y z of camera/artag output alligned with tryphon body frame 
//Eigen::Quaterniond quatang(0,1,0,0); //to make orientation all 0 when docking face alligned
//Rmatrix1=quatpos.toRotationMatrix();



/*
//////For Gazebo////////////
Eigen::Quaterniond quat(Pose1.orientation.w,Pose1.orientation.x,Pose1.orientation.y,Pose1.orientation.z);
Eigen::Quaterniond quattemp;
Eigen::Quaterniond quatpos(0, 0, .7071, .7071); //for gazebo
//quat=quat*quatang.inverse(); //allign offset when facin each other 
quattemp=quatpos*quat.inverse();
Rmatrix2=quattemp.toRotationMatrix();
*/


//ALVAR1position1=Rmatrix1*ALVAR1position1;


  //ALVAR2angle(0)=atan2(2*(quat.w()*quat.x()+quat.y()*quat.z()),1-2*(quat.x()*quat.x()+quat.y()*quat.y()));
  //ALVAR2angle(1)=asin(2*(quat.w()*quat.y()-quat.z()*quat.x()));
  //ALVAR2angle(2)=atan2(2*(quat.w()*quat.z()+quat.x()*quat.y()),1-2*(quat.z()*quat.z()+quat.y()*quat.y()));
  //Rmatrix2=quat.toRotationMatrix();
  ALVAR1pos(0)=ALVAR1position1(0);
  ALVAR1pos(1)=ALVAR1position1(1);
  ALVAR1pos(2)=ALVAR1position1(2);

/*
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
*/
}
	
void subALVAR1b(const geometry_msgs::PoseStamped Aposes1b) //camera frame on other tryphon
{
  
 
  geometry_msgs::Pose Pose1b=Aposes1b.pose ;
  //Pose=Aposes.pose;


  ALVAR1position2(0)=Pose1b.position.x; //  switch direction of vector
  ALVAR1position2(1)=Pose1b.position.y;
  ALVAR1position2(2)=Pose1b.position.z;
 
  
/////For Tryphon////////////
//Eigen::Quaterniond quatpos(.7071,-.7071,0,0); //to make x y z of camera/artag output alligned with tryphon body frame 
//Eigen::Quaterniond quatang(0,1,0,0); //to make orientation all 0 when docking face alligned
//Rmatrix1b=quatpos.toRotationMatrix();



/*
//////For Gazebo////////////
Eigen::Quaterniond quat(Pose1.orientation.w,Pose1.orientation.x,Pose1.orientation.y,Pose1.orientation.z);
Eigen::Quaterniond quattemp;
Eigen::Quaterniond quatpos(0, 0, .7071, .7071); //for gazebo
//quat=quat*quatang.inverse(); //allign offset when facin each other 
quattemp=quatpos*quat.inverse();
Rmatrix2=quattemp.toRotationMatrix();
*/


//ALVAR1position2=Rmatrix1*ALVAR1position2;


  //ALVAR2angle(0)=atan2(2*(quat.w()*quat.x()+quat.y()*quat.z()),1-2*(quat.x()*quat.x()+quat.y()*quat.y()));
  //ALVAR2angle(1)=asin(2*(quat.w()*quat.y()-quat.z()*quat.x()));
  //ALVAR2angle(2)=atan2(2*(quat.w()*quat.z()+quat.x()*quat.y()),1-2*(quat.z()*quat.z()+quat.y()*quat.y()));
  //Rmatrix2=quat.toRotationMatrix();

/*
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
*/
}


void subALVAR2a(const geometry_msgs::PoseStamped Aposes2) //camera frame on other tryphon
{
  
 
  geometry_msgs::Pose Pose2=Aposes2.pose ;
  //Pose=Aposes.pose;


  ALVAR2position1(0)=Pose2.position.x; //  switch direction of vector
  ALVAR2position1(1)=Pose2.position.y;
  ALVAR2position1(2)=Pose2.position.z;
 
  
/////For Tryphon////////////
//Eigen::Quaterniond quatpos(.7071,-.7071,0,0); //to make x y z of camera/artag output alligned with tryphon body frame 
//Eigen::Quaterniond quatang(0,1,0,0); //to make orientation all 0 when docking face alligned
//Rmatrix2=quatpos.toRotationMatrix();



/*
//////For Gazebo////////////
Eigen::Quaterniond quat(Pose1.orientation.w,Pose1.orientation.x,Pose1.orientation.y,Pose1.orientation.z);
Eigen::Quaterniond quattemp;
Eigen::Quaterniond quatpos(0, 0, .7071, .7071); //for gazebo
//quat=quat*quatang.inverse(); //allign offset when facin each other 
quattemp=quatpos*quat.inverse();
Rmatrix2=quattemp.toRotationMatrix();
*/


//ALVAR2position1=Rmatrix2*ALVAR2position1;


  //ALVAR2angle(0)=atan2(2*(quat.w()*quat.x()+quat.y()*quat.z()),1-2*(quat.x()*quat.x()+quat.y()*quat.y()));
  //ALVAR2angle(1)=asin(2*(quat.w()*quat.y()-quat.z()*quat.x()));
  //ALVAR2angle(2)=atan2(2*(quat.w()*quat.z()+quat.x()*quat.y()),1-2*(quat.z()*quat.z()+quat.y()*quat.y()));
  //Rmatrix2=quat.toRotationMatrix();
  ALVAR2pos(0)=ALVAR2position1(0);
  ALVAR2pos(1)=ALVAR2position1(1);
  ALVAR2pos(2)=ALVAR2position1(2);

/*
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
*/
}

void subALVAR2b(const geometry_msgs::PoseStamped Aposes2b) //camera frame on other tryphon
{
  
 
  geometry_msgs::Pose Pose2b=Aposes2b.pose ;
  //Pose=Aposes.pose;


  ALVAR2position2(0)=Pose2b.position.x; //  switch direction of vector
  ALVAR2position2(1)=Pose2b.position.y;
  ALVAR2position2(2)=Pose2b.position.z;
 
  
/////For Tryphon////////////
//Eigen::Quaterniond quatpos(.7071,-.7071,0,0); //to make x y z of camera/artag output alligned with tryphon body frame 
//Eigen::Quaterniond quatang(0,1,0,0); //to make orientation all 0 when docking face alligned
//Rmatrix2b=quatpos.toRotationMatrix();



/*
//////For Gazebo////////////
Eigen::Quaterniond quat(Pose1.orientation.w,Pose1.orientation.x,Pose1.orientation.y,Pose1.orientation.z);
Eigen::Quaterniond quattemp;
Eigen::Quaterniond quatpos(0, 0, .7071, .7071); //for gazebo
//quat=quat*quatang.inverse(); //allign offset when facin each other 
quattemp=quatpos*quat.inverse();
Rmatrix2=quattemp.toRotationMatrix();
*/


//ALVAR2position2=Rmatrix2b*ALVAR2position2;


  //ALVAR2angle(0)=atan2(2*(quat.w()*quat.x()+quat.y()*quat.z()),1-2*(quat.x()*quat.x()+quat.y()*quat.y()));
  //ALVAR2angle(1)=asin(2*(quat.w()*quat.y()-quat.z()*quat.x()));
  //ALVAR2angle(2)=atan2(2*(quat.w()*quat.z()+quat.x()*quat.y()),1-2*(quat.z()*quat.z()+quat.y()*quat.y()));
  //Rmatrix2=quat.toRotationMatrix();

/*
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
*/
}

void velocity_solver()
{
int i,j,k;
double mean[2][6];
double tempsumnum[2][6];
double tempsumden[2][6];
double tempsum[2][6];
/////Compute mean values


for (int i=0; i<2 ; i++)
{
	for (int j=0; j<6 ; j++)
	{
		for(k=0;k<10;k++)
		{
		pos[i][j][k]=pos[i][j][k+1];
		}
	}
}


for (i=0;i<2;i++)
{
	for(j=0;j<6;j++)
	{
	mean[i][j]=0;
		for(k=0;k<10;k++)
		{
		mean[i][j]=mean[i][j]+pos[i][j][k];
		}
	mean[i][j]=mean[i][j]/10.0;
	}
}
////

///compute velocity
for (i=0;i<2;i++)
{
	for(j=0;j<6;j++)
	{
		tempsumnum[i][j]=0.0;
		tempsumden[i][j]=0.0;
		tempsum[i][j]=0.0;
		for(k=0;k<10;k++)
		{
		tempsumnum[i][j]=tempsumnum[i][j]+((pos[i][j][k]-mean[i][j])*(posx[k]-meanx));
		tempsumden[i][j]=tempsumden[i][j]+pow(posx[k]-meanx,2);
		}
		tempsum[i][j]=tempsumnum[i][j]/tempsumden[i][j];
	}
}
////
		
	alvarv1.twist.linear.x=tempsum[0][0];
	alvarv1.twist.linear.y=tempsum[0][1];
	alvarv1.twist.linear.z=tempsum[0][2];
	alvarv1.twist.angular.x=tempsum[0][3];
	alvarv1.twist.angular.y=tempsum[0][4];
	alvarv1.twist.angular.z	=tempsum[0][5];
	
	alvarv2.twist.linear.x=tempsum[1][0];
	alvarv2.twist.linear.y=tempsum[1][1];
	alvarv2.twist.linear.z=tempsum[1][2];
	alvarv2.twist.angular.x=tempsum[1][3];
	alvarv2.twist.angular.y=tempsum[1][4];
	alvarv2.twist.angular.z	=tempsum[1][5];

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

ros::Subscriber sub1a = nh.subscribe("/192_168_10_243/ar_posea",1,subALVAR1a);
ros::Subscriber sub2a = nh.subscribe("/192_168_10_244/ar_posea",1,subALVAR2a);
ros::Subscriber sub1b = nh.subscribe("/192_168_10_243/ar_poseb",1,subALVAR1b);
ros::Subscriber sub2b = nh.subscribe("/192_168_10_244/ar_poseb",1,subALVAR2b);

ros::Publisher  alvarp1_pub = nh.advertise<geometry_msgs::PoseStamped>("/192_168_10_243/ar_pose1",1);
ros::Publisher  alvarp2_pub = nh.advertise<geometry_msgs::PoseStamped>("/192_168_10_243/ar_pose2",1);
ros::Publisher  alvarv1_pub = nh.advertise<geometry_msgs::TwistStamped>("/192_168_10_243/ar_vel1",1);
ros::Publisher  alvarv2_pub = nh.advertise<geometry_msgs::TwistStamped>("/192_168_10_243/ar_vel2",1);

ros::Rate loop_rate(10);

pose_zero(alvarp1);
pose_zero(alvarp2);
twist_zero(alvarv1);
twist_zero(alvarv2);

posx[0]=0.0; //set up
for (int a=0; a<9; a++)
{
posx[a+1]=posx[a]+0.1;
}

for (int i=0; i<6 ; i++)
{
	for (int j=0; j<10 ; j++)
	{
		pos[0][i][j]=0;
		pos[1][i][j]=0;
		
	}

}

while (ros::ok())
 {

ros::spinOnce();


////////////////////find orientation/////////////////////////////
//roll  (x axis = pitch in docking situation)
ALVAR1angle(0)=atan2(ALVAR1position2(1)-ALVAR1position1(1),.15); //of tag set 1
ALVAR2angle(0)=atan2(ALVAR2position2(1)-ALVAR2position1(1),.15);  //of tag set 2
ALVARangle(0)=.5*ALVAR1angle(0)+.5*ALVAR2angle(0);

//pitch (y axis = roll in docking situation)
ALVAR1angle(1)=atan2(ALVAR2position1(2)-ALVAR1position1(2),2.1); //of main tag, --> 2 is distance between tags
ALVAR2angle(1)=atan2(ALVAR2position2(2)-ALVAR1position2(2),2.1);  //of extra tag
ALVARangle(1)=.5*ALVAR1angle(1)+.5*ALVAR2angle(1);

//yaw  
ALVAR1angle(2)=-atan2(ALVAR2position1(1)-ALVAR1position1(1),2.1);//of main tag (-) for proper orientation
ALVAR2angle(2)=-atan2(ALVAR2position2(1)-ALVAR1position2(1),2.1); //of extra tag
ALVARangle(2)=.5*ALVAR1angle(2)+.5*ALVAR2angle(2);

////////////////////////////////////////////////////////////////

//////compute velocity////////


	pos[0][0][10]=ALVAR1pos(0);
	pos[0][1][10]=ALVAR1pos(1);
	pos[0][2][10]=ALVAR1pos(2);
	pos[0][3][10]=ALVAR1angle(0);
	pos[0][4][10]=ALVAR1angle(1);
	pos[0][5][10]=ALVAR1angle(2);	

	pos[1][0][10]=ALVAR2pos(0);
	pos[1][1][10]=ALVAR2pos(1);
	pos[1][2][10]=ALVAR2pos(2);
	pos[1][3][10]=ALVAR2angle(0);
	pos[1][4][10]=ALVAR2angle(1);
	pos[1][5][10]=ALVAR2angle(2);	


velocity_solver();


//remove angle difference when facing each other
//ALVARangle=quatdiff.toRotationMatrix()*ALVARangle;



alvarp1.header.stamp=ros::Time::now();
alvarp2.header.stamp=ros::Time::now();
alvarp1.pose=vects2pose(ALVAR1pos,ALVARangle);
alvarp2.pose=vects2pose(ALVAR2pos,ALVARangle);


alvarv1.header.stamp=ros::Time::now();
alvarv2.header.stamp=ros::Time::now();

  	alvarp1_pub.publish(alvarp1);
  	alvarp2_pub.publish(alvarp2);
  	alvarv1_pub.publish(alvarv1);
  	alvarv2_pub.publish(alvarv2);


loop_rate.sleep();
}

	return 0;
}



