



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

