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


//using namespace Eigen;
Eigen::Vector3d rel_pos_1,rel_ang_1, rel_vel_1, rel_avel_1;
Eigen::Vector3d rel_pos_2,rel_ang_2, rel_vel_2, rel_avel_2;
Eigen::Vector3d force_1,force_2,torque_1,torque_2;

Eigen::Vector3d r1(-1.05,1.128,0); //cm to camera 1 
Eigen::Vector3d r2(1.05,1.128,0);  //cm to camera 2


Eigen::MatrixXd Kgain(6,18);
Eigen::VectorXd statevect(18);
Eigen::VectorXd forcevect(6);

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
	rel_pos_2(1)=rel_pos_2(1)-0.25; 
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

ros::Subscriber  pose1_sub= nh.subscribe("/192_168_10_243/ar_pose1",1,p1_callback);
ros::Subscriber  pose2_sub= nh.subscribe("/192_168_10_243/ar_pose2",1,p2_callback);
ros::Subscriber  vel1_sub= nh.subscribe("/192_168_10_243/ar_vel1",1,v1_callback);
ros::Subscriber  vel2_sub= nh.subscribe("/192_168_10_243/ar_vel2",1,v2_callback);

ros::Publisher  wrench1_pub = nh.advertise<geometry_msgs::WrenchStamped>("/dock_wrench_1",1);
ros::Publisher  wrench2_pub = nh.advertise<geometry_msgs::WrenchStamped>("/dock_wrench_2",1);
ros::Publisher  netwrench_pub = nh.advertise<geometry_msgs::Wrench>("/192_168_10_243/command_control",1); //to work currently with FD


ros::Rate loop_rate(10);


Kgain<<             .69,0,0,.69,0,0,0,0,0,5.75,0,0,5.75,0,0,0,0,0,
					0,1.0580,0,0,1.0580,0,0,0,0,0,7.2450,0,0,7.245,0,0,0,0,
					0,0,4.1,0,0,4.1,0,0,0,0,0,14.15,0,0,14.15,0,0,0,
					0,0,0,0,0,0,.3,0,0,0,0,0,0,0,0,4.5,0,0,
					0,0,-.315,0,0,.315,0,.315,0,0,0,-2.55,0,0,2.55,0,2.55,0,
					0,1,0,0,-1,0,0,0,1,0,6,0,0,-6,0,0,0,6;


/*
Kgain<<             .69,0,0,.69,0,0,0,0,0,5.75,0,0,5.75,0,0,0,0,0,
					0,1.0580,0,0,1.0580,0,0,0,0,0,7.2450,0,0,7.245,0,0,0,0,
					0,0,4.37,0,0,4.37,0,0,0,0,0,24.15,0,0,24.15,0,0,0,
					0,0,0,0,0,0,.3,0,0,0,0,0,0,0,0,4.5,0,0,
					0,0,-.315,0,0,.315,0,.315,0,0,0,-2.55,0,0,2.55,0,2.55,0,
					0,1,0,0,-1,0,0,0,1,0,6,0,0,-6,0,0,0,6;

*/

while (ros::ok())
 {

ros::spinOnce();
statevect<<rel_pos_1,rel_pos_2,rel_ang_1,rel_vel_1,rel_vel_2,rel_avel_1;
forcevect=Kgain*statevect;

for (int i=0; i<3 ; i++)
{
force_1(i)=forcevect(i);
torque_1(i)=forcevect(i+3);
}
netwrench=vects2wrench(force_1,torque_1);
netwrench_pub.publish(netwrench);
 loop_rate.sleep();

 }


return 0;
}