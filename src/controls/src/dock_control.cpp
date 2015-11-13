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
#include <controls/dock_controlConfig.h>


#include "controls/Commands.h"
#include <Euler_utility.h>
#include "vects2geoMsgs.cpp"





//using namespace Eigen;
Eigen::Vector3d rel_pos_1,rel_ang_1, rel_vel_1, rel_avel_1;
Eigen::Vector3d rel_pos_2,rel_ang_2, rel_vel_2, rel_avel_2;
Eigen::Vector3d force,torque;

Eigen::Vector3d r1(-1.05,1.128,0); //cm to camera 1 
Eigen::Vector3d r2(1.05,1.128,0);  //cm to camera 2
Eigen::Vector3d stabilizepos(3);
Eigen::Vector3d stabilizeang(3);
Eigen::MatrixXd Kgain(6,18);
Eigen::VectorXd statevect(18),stateerr(18);
Eigen::VectorXd forcevect(6);

geometry_msgs::WrenchStamped wrench_1, wrench_2;
geometry_msgs::Wrench netwrench;
ros::Publisher MaxPrct_node;
 
 bool path_debut=true;
double path_debut_time=0;
 double intZ = 0;
 double inty = 0;
 double a_factor=.01; //glideslope <0, should probably be some function of initial distance since it controls the period
double a;
double x_initial; //should actually be once the tryphons are alligned
double v_max; //maximum allowed velocity upon docking
double v_init; 
double period;
int dock_ready=0;
std_msgs::Float64 maxPrctThrust;

//coming in body frame
void p1_callback(const geometry_msgs::PoseStamped p1)
{
    rel_pos_1=pose2vect_pos(p1.pose);
     rel_pos_1(1)=rel_pos_1(1)+0.25; //taking into account docking mechanism distance if put in -y
   // rel_pos_1(1)=rel_pos_1(1)-0.25; //taking into account docking mechanism distance if put in +y
    rel_ang_1=pose2vect_angle(p1.pose);
}


void p2_callback(const geometry_msgs::PoseStamped p2)
{
	rel_pos_2=pose2vect_pos(p2.pose);
  rel_pos_2(1)=rel_pos_2(1)+0.25; //taking into account docking mechanism distance if put in -y
   // rel_pos_2(1)=rel_pos_2(1)-0.25; //taking into account docking mechanism distance if put in +y 
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


void callback(controls::dock_controlConfig &config, uint32_t level) 
{
	ROS_INFO("Reconfigure Request: dockready:%d",config.dockready);

dock_ready=config.dockready;
}

/*
void glideslope()
{

	double t=0;
	double r=0;
	double r_dot=0;
	double bearingx;
	double bearingz;

	if(path_debut)
	{
	path_debut_time=ros::Time::now().toSec();
   
				x_initial=-sqrt(pow(rel_pos_1(0),2.0)+pow(rel_pos_1(1),2.0)+pow(rel_pos_1(2),2.0);
				plane_xy=sqrt(pow(rel_pos_1(0),2.0)+pow(rel_pos_1(1),2.0));
				plane_yz=sqrt(pow(rel_pos_1(2),2.0)+pow(rel_pos_1(1),2.0));
				a=(a_factor*x_initial); //define slope as a function of x_initial, (a distance of 4 m should take more time for docking than 3m) //maximum allowed velocity upon docking
				v_max=.02;
				v_init=(a*x_initial+v_max); 
				period=1.0/a*log(v_max/v_init); //The glideslope algorithm has a period (TBD)
				ROS_INFO("Period=%f, V_init=%f, a=%f, x_inital=%f", period, v_init, a ,x_initial); //this must be possible with force of thrusters
				//magnet_on.data=true;
				path_debut=false;
				bearingx=acos(rel_pos_1(0)/plane_xy);
				bearingz=acos(rel_pos_1(1)/plane_yz);

	}

t= ros::Time::now().toSec() - path_debut_time;
r=x_initial*exp(a*t)+v_max/a*(exp(a*t)-1); //-(sidelength_cube-wiggle_room*(1-exp(-t))) //x_initial should be defined as distance from //subtract sidelength from r so that final distance is fromf aces and not center of masses
r_dot=a*r+v_max;


}
*/

int main(int argc, char **argv)
{

double KpP[3],KdP[3],KpT[3],KdT[3];
Eigen::Matrix3d RCPM1,RCPM2;
 Eigen::Vector3d vecZ;
 vecZ(0)=0;
  vecZ(1)=0;  
  vecZ(2)=0.0141;
maxPrctThrust.data=50.0;

RCPM1=CPM(r1);
RCPM2=CPM(r2);

for (int k = 0 ; k<3;k++){stabilizepos(k)=0;};
for (int k = 0 ; k<3;k++){stabilizeang(k)=0;};
stabilizepos(1)=-1.2;

ros::init(argc, argv, "dock_control");
ros::NodeHandle nh;

ros::Subscriber  pose1_sub= nh.subscribe("/192_168_10_243/ar_pose1",1,p1_callback);
ros::Subscriber  pose2_sub= nh.subscribe("/192_168_10_243/ar_pose2",1,p2_callback);
ros::Subscriber  vel1_sub= nh.subscribe("/192_168_10_243/ar_vel1",1,v1_callback);
ros::Subscriber  vel2_sub= nh.subscribe("/192_168_10_243/ar_vel2",1,v2_callback);

ros::Publisher  wrench1_pub = nh.advertise<geometry_msgs::WrenchStamped>("/propForce",1);
ros::Publisher  wrench2_pub = nh.advertise<geometry_msgs::WrenchStamped>("/derivForce",1);
ros::Publisher  netwrench_pub = nh.advertise<geometry_msgs::Wrench>("/192_168_10_243/command_control",1); //to work currently with FD
MaxPrct_node = nh.advertise<std_msgs::Float64>("/192_168_10_243/max_thrust",1);


dynamic_reconfigure::Server<controls::dock_controlConfig> server;
dynamic_reconfigure::Server<controls::dock_controlConfig>::CallbackType f;

 f = boost::bind(&callback, _1, _2);
 server.setCallback(f);

ros::Rate loop_rate(10);



Kgain<<             0.1828,0,0,0.1828,0,0,0,0,0,1.9785,0,0,1.9785,0,0,0,0,0, //gains signs changed for negative y cameras
					0,0.1463,0,0,0.1463,0,0,0,0,0,1.7485,0,0,1.7485,0,0,0,0,
					0,0,.7704,0,0,.7704,0,0.,0.,0,0,2.0500,0,0,2.0500,0,0,0,
					0,0,0,0,0,0,0.12,0,0,0,0,0,0,0,0,1.7000,0,0,
					0,0,0.145,0,0,-0.145,0,0.145,0,0,0,1.95,0,0,-1.95,0,1.95,0,
					0,-0.175,0,0,0.175,0,0,0,0.175,0,-1.05,0,0,1.05,0,0,0,1.05;

/*
Kgain<<             0.1828,0,0,0.1828,0,0,0,0,0,1.9785,0,0,1.9785,0,0,0,0,0,  //good gains but with +y cameras
					0,0.1463,0,0,0.1463,0,0,0,0,0,1.7485,0,0,1.7485,0,0,0,0,
					0,0,0.2704,0,0,0.2704,0,0.,0.,0,0,2.0500,0,0,2.0500,0,0,0,
					0,0,0,0,0,0,0.12,0,0,0,0,0,0,0,0,1.7000,0,0,
					0,0,-0.145,0,0,0.145,0,0.145,0,0,0,-1.95,0,0,1.95,0,1.95,0,
					0,0.175,0,0,-0.175,0,0,0,0.175,0,1.05,0,0,-1.05,0,0,0,1.05;

	

	Kgain<<             0.1828,0,0,0.1828,0,0,0,0,0,2.9785,0,0,2.9785,0,0,0,0,0,
						0,0.1463,0,0,0.1463,0,0,0,0,0,2.7485,0,0,2.7485,0,0,0,0,
						0,0,0.2704,0,0,0.2704,0,0.,0.,0,0,5.0500,0,0,5.0500,0,0,0,
						0,0,0,0,0,0,0.12,0,0,0,0,0,0,0,0,2.7000,0,0,
						0,0,-0.145,0,0,0.145,0,0.145,0,0,0,-1.95,0,0,1.95,0,1.95,0,
						0,0.075,0,0,-0.075,0,0,0,0.075,0,1.05,0,0,-1.05,0,0,0,1.05;




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
	
	


if(!dock_ready)
{
statevect<<rel_pos_1-stabilizepos,rel_pos_2-stabilizepos,rel_ang_1-stabilizeang,rel_vel_1,rel_vel_2,rel_avel_1;
}


if(dock_ready)	
{
statevect<<rel_pos_1,rel_pos_2,rel_ang_1,rel_vel_1,rel_vel_2,rel_avel_1;
//glideslope();
}

	forcevect=0.9*Kgain*statevect;
	for (int i=0; i<3 ; i++)
	{
	force(i)=forcevect(i);
//torque(i)=forcevect(i+3);
	}
torque(2)=forcevect(5);
 if(fabs(force(2))<2.4 && fabs(force(1))<1.2 && fabs(force(0))<1.2) // increasing only if the command is not saturating //
     {
        intZ+= (rel_pos_2(2)+rel_pos_1(2))/20; //integral term in z, /20 because taking average of both measurements
        if(fabs(intZ*vecZ(2))>0.6){intZ=copysign(0.6/vecZ(2),intZ);}

      }

//compute prop and deriv terms seperatly
wrench_1.header.stamp=ros::Time::now();
wrench_2.header.stamp=ros::Time::now();
wrench_1.wrench.force.x=0.6*(.1828*rel_pos_1(0)+.1828*rel_pos_2(0));
wrench_2.wrench.force.x=0.6*(1.9785*rel_vel_1(0)+1.9785*rel_vel_2(0));
wrench_1.wrench.force.y=0.6*(.1463*rel_pos_1(1)+.1463*rel_pos_2(1));
wrench_2.wrench.force.y=0.6*(1.7485*rel_vel_1(1)+1.7485*rel_vel_2(1));
wrench_1.wrench.force.z=0.6*(0.2704*rel_pos_1(2)+0.2704*rel_pos_2(2));
wrench_2.wrench.force.z=0.6*(2.0500*rel_vel_1(2)+2.0500*rel_vel_2(2));
wrench_1.wrench.torque.z=0.6*(0.075*rel_pos_1(1)-0.075*rel_pos_2(1)+0.075*rel_ang_1(2));
wrench_2.wrench.torque.z=0.6*(1.05*rel_vel_1(1)-1.05*rel_vel_2(1)+1.05*rel_avel_1(2));

wrench_1=wrench_1;
wrench_1=wrench_2;


MaxPrct_node.publish(maxPrctThrust);
force(2)=force(2)+0.1*intZ*vecZ(2);
wrench1_pub.publish(wrench_1);
wrench2_pub.publish(wrench_2);
netwrench=vects2wrench(force,torque);
netwrench_pub.publish(netwrench);

 loop_rate.sleep();

 }


return 0;
}