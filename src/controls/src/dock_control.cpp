#include "controls.h"
#include "ctr_fuzzy.h"
/*#define DEVICE_I2C      "/dev/i2c-3"
typedef enum {FALSE, TRUE}
BOOLEAN;
typedef unsigned char BYTE;
#define INTERACTION     0
*/
#include "controls/State.h"

#include <vector>
#include <cmath>

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <net/if.h>
#include <arpa/inet.h>
#include <signal.h>
#include <iostream>

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/WrenchStamped.h"
#include <dynamic_reconfigure/server.h>
#include <controls/controlConfig.h>


#include "controls/Commands.h"
#include <Euler_utility.h>
#include "vects2geoMsgs.cpp"



Eigen::Vector3d rel_pos_1,rel_ang_1, rel_vel_1, rel_avel_1;
Eigen::Vector3d rel_pos_2,rel_ang_2, rel_vel_2, rel_avel_2;
Eigen::Vector3d force_1,force_2,torque_1,torque_2;

Eigen::Vector3d r1(-1.05,1.128,0); //cm to camera 1 
Eigen::Vector3d r2(1.05,1.128,0);  //cm to camera 2


geometry_msgs::WrenchStamped wrench_1, wrench_2;
geometry_msgs::Wrench netwrench;


//coming in body frame
void p1_callback(const geometry_msgs::PoseStamped p1)
{
    rel_pos_1=pose2vect_pos(p1.pose);
    rel_pos_1(1)=rel_pos_1(1)-0.25; //taking into account docking mechanism distance
    rel_ang_1=pose2vect_angle(p1.pose);
}


void p2_callback(const geometry_msgs::PoseStamped p2)
{
	rel_pos_2=pose2vect_pos(p2.pose);
    rel_ang_2=pose2vect_angle(p2.pose);

}

void v1_callback(const geometry_msgs::TwistStamped v1)
{
 	rel_vel_1=twist2vect_linear(v1.twist);
    rel_avel_1=twist2vect_angular(v1.twist);

}

void v2_callback(const geometry_msgs::TwistStamped v2)
{
	rel_vel_2=twist2vect_linear(v2.twist);
    rel_avel_2=twist2vect_angular(v2.twist);
}

Eigen::Matrix3d CPM(Eigen::Vector3d vect) // return cross product matrix
{
	Eigen::Matrix3d CPM;
	CPM<<       0,  -vect(2),   vect(1),
		  vect(2),         0,  -vect(0),
		  -vect(1),  vect(0),         0;

	return CPM;
}

int main(int argc, char **argv)
{

double KpP[3],KdP[3],KpT[3],KdT[3];
Eigen::Matrix3d RCPM1,RCPM2;

RCPM1=CPM(r1);
RCPM2=CPM(r2);


ros::init(argc, argv, "dock_control");
ros::NodeHandle nh;

ros::Subscriber  pose1_sub= nh.subscribe("/192_168_10_243/ar_pose",1,p1_callback);
ros::Subscriber  pose2_sub= nh.subscribe("/192_168_10_244/ar_pose",1,p2_callback);
ros::Subscriber  vel1_sub= nh.subscribe("/192_168_10_243/ar_vel",1,v1_callback);
ros::Subscriber  vel2_sub= nh.subscribe("/192_168_10_244/ar_vel",1,v2_callback);

ros::Publisher  wrench1_pub = nh.advertise<geometry_msgs::WrenchStamped>("/dock_wrench_1",1);
ros::Publisher  wrench2_pub = nh.advertise<geometry_msgs::WrenchStamped>("/dock_wrench_2",1);
ros::Publisher  netwrench_pub = nh.advertise<geometry_msgs::Wrench>("/192_168_10_243/command_control",1); //to work currently with FD


ros::Rate loop_rate(10);


KpP[0]=.2;
KpP[1]=.1;
KpP[2]=.2;
KpT[0]=.25;
KpT[1]=.8;
KpT[2]=.25;

/*
KdP[0]=0;
KdP[1]=0;
KdP[2]=0;
KdT[0]=0;
KdT[1]=0;
KdT[2]=0;
*/

KdP[0]=.5;
KdP[1]=.5;
KdP[2]=.5;
KdT[0]=.2;
KdT[1]=.2;
KdT[2]=.2;



while (ros::ok())
 {

ros::spinOnce();

for (int i=0; i<3 ; i++)
{
force_1(i)=KpP[i]*rel_pos_1(i)+KdP[i]*rel_vel_1(i); //think about (-)
force_2(i)=KpP[i]*rel_pos_2(i)+KdP[i]*rel_vel_2(i);


//torque_1(i)=-KpT[i]*rel_ang_1(i)-KdT[i]*rel_avel_1(i); //think about (-)  <---
//torque_2(i)=-KpT[i]*rel_ang_2(i)-KdT[i]*rel_avel_2(i);

torque_1(i)=0;
torque_2(i)=0;
}


//torque_1(0)=KpT[0]/2*-(rel_pos_2(2)+rel_pos_1(2)) 
torque_1(1)=KpT[1]*-(rel_pos_2(2)-rel_pos_1(2));
torque_1(2)=KpT[2]*(rel_pos_1(1)-rel_pos_2(1));
//torque_2(0)=
torque_2(1)=KpT[1]*-(rel_pos_2(2)-rel_pos_1(2));
torque_2(2)=KpT[2]*(rel_pos_2(1)-rel_pos_1(1));



wrench_1.header.stamp=ros::Time::now();
wrench_2.header.stamp=ros::Time::now();
wrench_1.wrench=vects2wrench(force_1,torque_1); //wrench before CM change
wrench_2.wrench=vects2wrench(force_2,torque_2);

//moving to CM
//torque_1=torque_1+RCPM1*force_1;
//torque_2=torque_2+RCPM2*force_2;

//sharing functions

netwrench=vects2wrench(.5*force_1+.5*force_2,.5*torque_1+.5*torque_2);






wrench1_pub.publish(wrench_1); //wrench at camera frame origin
wrench2_pub.publish(wrench_2);
netwrench_pub.publish(netwrench);
 }


return 0;
}