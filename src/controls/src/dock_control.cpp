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
Eigen::Vector3d glidepos1,glidepos2,glidevel1,glidevel2;
Eigen::Vector3d rho1(-1.05,1.128,0); //cm to camera 1 
Eigen::Vector3d rho2(1.05,1.128,0);  //cm to camera 2
Eigen::Vector3d stabilizepos(3);
Eigen::Vector3d stabilizeang(3);
Eigen::MatrixXd Kgain(6,18);
Eigen::VectorXd statevect(18),stateerr(18);
Eigen::VectorXd forcevect(6);

geometry_msgs::WrenchStamped wrench_1, wrench_2,glidePS1,glidePS2;
geometry_msgs::Wrench netwrench;
ros::Publisher MaxPrct_node;
 
 bool path_debut1=true;
 bool path_debut2=true;
double path_debut_time1=0;
double path_debut_time2=0;
 double intZ = 0;
 double inty = 0;
 double x_initial1;
 double x_initial2;
double a=-0.05;//(a_factor*x_initial); //define slope as a function of x_initial, (a distance of 4 m should take more time for docking than 3m) //maximum allowed velocity upon docking
double v_max=-.02; 
double t1=0;
double r1=0;
double r_dot1=0;
double t2=0;
double r2=0;
double r_dot2=0;

int dock_ready=0;
int glide_ready=1;
std_msgs::Float64 maxPrctThrust;

//coming in body frame
void p1_callback(const geometry_msgs::PoseStamped p1)
{
    rel_pos_1=pose2vect_pos(p1.pose);
     // rel_pos_1(1)=rel_pos_1(1)+0.25; //taking into account docking mechanism distance if put in -y
    //taking into account docking mechanism distance if put in +y
   rel_pos_1(0)=rel_pos_1(0)-0.25*sin(rel_ang_1(2)) ; //ignoring pitch and roll .25 in target frame
   rel_pos_1(1)=rel_pos_1(1)-0.25*cos(rel_ang_1(2));
    rel_ang_1=pose2vect_angle(p1.pose);
}


void p2_callback(const geometry_msgs::PoseStamped p2)
{
	rel_pos_2=pose2vect_pos(p2.pose);
  //rel_pos_2(1)=rel_pos_2(1)+0.25; //taking into account docking mechanism distance if put in -y
 rel_pos_2(0)=rel_pos_2(0)-0.25*sin(rel_ang_2(2)) ;
   rel_pos_2(1)=rel_pos_2(1)-0.25*cos(rel_ang_2(2)); //taking into account docking mechanism distance if put in +y 
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
glide_ready=config.glideready;
stabilizepos(1)=config.stab_pos;
}


void glideslope1() //can make y's differences be a theta desired
{

	double period,v_init,rcurrent1;
	
	if(path_debut1)
	{
		
	path_debut_time1=ros::Time::now().toSec();
				x_initial1=(double)sqrt(pow(rel_pos_1(0),2.0)+pow(rel_pos_1(1),2.0)+pow(rel_pos_1(2),2.0)); //for -y works, if +y, r is now pos and vel is -, so v_max =(-), with x_init(+) and a(-) 
				v_init=(double)(a*x_initial1+v_max); 
				period=1.0/a*log(v_max/v_init); //The glideslope algorithm has a period (TBD)
				//ROS_INFO("Period=%f, V_init=%f, a=%f, x_inital=%f", period, v_init, a ,x_initial); //this must be possible with force of thrusters
				//magnet_on.data=true;
				path_debut1=false;
	ROS_INFO("period=%f",period);
	}

t1= ros::Time::now().toSec() - path_debut_time1;
ROS_INFO("t=%f",t1);
r1=x_initial1*exp(a*t1)+v_max/a*(exp(a*t1)-1); //-(sidelength_cube-wiggle_room*(1-exp(-t))) //x_initial should be defined as distance from //subtract sidelength from r so that final distance is fromf aces and not center of masses
r_dot1=a*r1+v_max;
rcurrent1=sqrt(pow(rel_pos_1(0),2)+pow(rel_pos_1(1),2)+pow(rel_pos_1(2),2));
if(t1<=period)
{
	ROS_INFO("hi");
glidepos1(0)=r1*(rel_pos_1(0)/fabs(x_initial1));
glidepos1(1)=r1*(rel_pos_1(1)/fabs(x_initial1));
glidepos1(2)=r1*(rel_pos_1(2)/fabs(x_initial1));
glidevel1(0)=r_dot1*(rel_pos_1(0)/fabs(x_initial1));
glidevel1(1)=r_dot1*(rel_pos_1(1)/fabs(x_initial1));
glidevel1(2)=r_dot1*(rel_pos_1(2)/fabs(x_initial1));
}
else
{
glidepos1(0)=0;
glidepos1(1)=0;
glidepos1(2)=0;
glidevel1(0)=0;
glidevel1(1)=0;
glidevel1(2)=0;	
}


}

void glideslope2()
{	

double period,v_init,rcurrent2;

	if(path_debut2)
	{
	path_debut_time2=ros::Time::now().toSec();
   
				x_initial2=sqrt(pow(rel_pos_2(0),2.0)+pow(rel_pos_2(1),2.0)+pow(rel_pos_2(2),2.0)); //for -y works, if +y, r is now pos and vel is -, so v_max =(-), with x_init(+) and a(-) 
				v_init=(a*x_initial2+v_max); 
				period=1.0/a*log(v_max/v_init); //The glideslope algorithm has a period (TBD)
				//ROS_INFO("Period=%f, V_init=%f, a=%f, x_inital=%f", period, v_init, a ,x_initial); //this must be possible with force of thrusters
				//magnet_on.data=true;
				path_debut2=false;
	
	}

t2= ros::Time::now().toSec() - path_debut_time2;
r2=x_initial2*exp(a*t2)+v_max/a*(exp(a*t2)-1); //-(sidelength_cube-wiggle_room*(1-exp(-t))) //x_initial should be defined as distance from //subtract sidelength from r so that final distance is fromf aces and not center of masses
r_dot2=a*r2+v_max;
rcurrent2=sqrt(pow(rel_pos_2(0),2)+pow(rel_pos_2(1),2)+pow(rel_pos_2(2),2));

if(t2<=period)
{
glidepos2(0)=r2*(rel_pos_1(0)/fabs(x_initial2));
glidepos2(1)=r2*(rel_pos_1(1)/fabs(x_initial2));
glidepos2(2)=r2*(rel_pos_1(2)/fabs(x_initial2)); //(-) for negative y
glidevel2(0)=r_dot2*(rel_pos_1(0)/fabs(x_initial2));
glidevel2(1)=r_dot2*(rel_pos_1(1)/fabs(x_initial2));
glidevel2(2)=r_dot2*(rel_pos_1(2)/fabs(x_initial2));
}
else
{
glidepos2(0)=0;
glidepos2(1)=0+0.25;//(-) for negative y
glidepos2(2)=0;
glidevel2(0)=0;
glidevel2(1)=0;
glidevel2(2)=0;	
}

}

int main(int argc, char **argv)
{

double KpP[3],KdP[3],KpT[3],KdT[3];
Eigen::Matrix3d RCPM1,RCPM2;
 Eigen::Vector3d vecZ;
 vecZ(0)=0;
  vecZ(1)=0;  
  vecZ(2)=0.05;
maxPrctThrust.data=100.0;



glidepos1(0)=0;
glidepos1(1)=0;//(-) for negative y
glidepos1(2)=0;
glidevel1(0)=0;
glidevel1(1)=0;
glidevel1(2)=0;
glidepos2(0)=0;
glidepos2(1)=0;//(-) for negative y
glidepos2(2)=0;
glidevel2(0)=0;
glidevel2(1)=0;
glidevel2(2)=0;

RCPM1=CPM(rho1);
RCPM2=CPM(rho2);

for (int k = 0 ; k<3;k++){stabilizepos(k)=0;};
for (int k = 0 ; k<3;k++){stabilizeang(k)=0;};
stabilizepos(1)=.6; //(-) for -y cameras//need to include rotation

ros::init(argc, argv, "dock_control");
ros::NodeHandle nh;

ros::Subscriber  pose1_sub= nh.subscribe("/192_168_10_243/ar_pose1",1,p1_callback);
ros::Subscriber  pose2_sub= nh.subscribe("/192_168_10_243/ar_pose2",1,p2_callback);
ros::Subscriber  vel1_sub= nh.subscribe("/192_168_10_243/ar_vel1",1,v1_callback);
ros::Subscriber  vel2_sub= nh.subscribe("/192_168_10_243/ar_vel2",1,v2_callback);

ros::Publisher  wrench1_pub = nh.advertise<geometry_msgs::WrenchStamped>("/propForce",1);
ros::Publisher  wrench2_pub = nh.advertise<geometry_msgs::WrenchStamped>("/derivForce",1);
ros::Publisher  glide1_pub = nh.advertise<geometry_msgs::WrenchStamped>("/glide1",1);
ros::Publisher  glide2_pub = nh.advertise<geometry_msgs::WrenchStamped>("/glide2",1);
ros::Publisher  netwrench_pub = nh.advertise<geometry_msgs::Wrench>("/192_168_10_243/command_control",1); //to work currently with FD
MaxPrct_node = nh.advertise<std_msgs::Float64>("/192_168_10_243/max_thrust",1);


dynamic_reconfigure::Server<controls::dock_controlConfig> server;
dynamic_reconfigure::Server<controls::dock_controlConfig>::CallbackType f;

 f = boost::bind(&callback, _1, _2);
 server.setCallback(f);

ros::Rate loop_rate(10);

/*
Kgain<<             0.6828,0,0,0.6828,0,0,0,0,0,3.8785,0,0,3.8785,0,0,0,0,0,  //good gains but with +y cameras
					0,0.5463,0,0,0.5463,0,0,0,0,0,2.9485,0,0,2.9485,0,0,0,0,
					0,0,1.15,0,0,1.15,0,0.,0.,0,0,5.5500,0,0,5.5500,0,0,0,
					0,0,0,0,0,0,1.05,0,0,0,0,0,0,0,0,4.7000,0,0,
					0,0,-0.145,0,0,0.145,0,0.145,0,0,0,0,0,0,0,0,0,0,
					0,0.175,0,0,-0.175,0,0,0,0.175,0,1.05,0,0,-1.05,0,0,0,1.05;

*/

/*


	Kgain<<             0.1828,0,0,0.1828,0,0,0,0,0,1.9785,0,0,1.9785,0,0,0,0,0, //gains signs changed for negative y cameras
					0,0.1463,0,0,0.1463,0,0,0,0,0,1.7485,0,0,1.7485,0,0,0,0,
					0,0,.7704,0,0,.7704,0,0.,0.,0,0,2.0500,0,0,2.0500,0,0,0,
					0,0,0,0,0,0,0.12,0,0,0,0,0,0,0,0,1.7000,0,0,
					0,0,0.145,0,0,-0.145,0,0.145,0,0,0,1.95,0,0,-1.95,0,1.95,0,
					0,-0.175,0,0,0.175,0,0,0,0.175,0,-1.05,0,0,1.05,0,0,0,1.05;

	Kgain<<             0.1828,0,0,0.1828,0,0,0,0,0,2.9785,0,0,2.9785,0,0,0,0,0,
						0,0.1463,0,0,0.1463,0,0,0,0,0,2.7485,0,0,2.7485,0,0,0,0,
						0,0,0.2704,0,0,0.2704,0,0.,0.,0,0,5.0500,0,0,5.0500,0,0,0,
						0,0,0,0,0,0,0.12,0,0,0,0,0,0,0,0,2.7000,0,0,
						0,0,-0.145,0,0,0.145,0,0.145,0,0,0,-1.95,0,0,1.95,0,1.95,0,
						0,0.075,0,0,-0.075,0,0,0,0.075,0,1.05,0,0,-1.05,0,0,0,1.05;

*/


	Kgain<<             .69,0,0,.69,0,0,0,0,0,5.75,0,0,5.75,0,0,0,0,0,
						0,1.0580,0,0,1.0580,0,0,0,0,0,7.2450,0,0,7.245,0,0,0,0,
						0,0,4.37,0,0,4.37,0,0,0,0,0,24.15,0,0,24.15,0,0,0,
						0,0,0,0,0,0,.3,0,0,0,0,0,0,0,0,4.5,0,0,
						0,0,-.315,0,0,.315,0,.315,0,0,0,-2.55,0,0,2.55,0,2.55,0,
						0,1,0,0,-1,0,0,0,1,0,6,0,0,-6,0,0,0,6;

	

	while (ros::ok())
	 {

	ros::spinOnce();
	
	


if(!dock_ready)
{
stateerr<<rel_pos_1-stabilizepos,rel_pos_2-stabilizepos,rel_ang_1-stabilizeang,rel_vel_1,rel_vel_2,rel_avel_1;
}


if(dock_ready)	
{
//statevect<<rel_pos_1,rel_pos_2,rel_ang_1,rel_vel_1,rel_vel_2,rel_avel_1;
if(glide_ready)
{
glideslope1();
glideslope2();
}
stateerr<<rel_pos_1-glidepos1,rel_pos_2-glidepos2,rel_ang_1,rel_vel_1-glidevel1,rel_vel_2-glidevel2,rel_avel_1;
}

	forcevect=0.9*Kgain*stateerr;
	for (int i=0; i<3 ; i++)
	{
	force(i)=forcevect(i);
//torque(i)=forcevect(i+3);
	}
torque(2)=forcevect(5);
//torque(1)=forcevect(4); pitch in body frame (roll about y)
//torque(0)=forcevect(3); //ro//ll in body frame pitch for y cam)

 if(fabs(force(2))<2.4 && fabs(force(1))<1.2 && fabs(force(0))<1.2) // increasing only if the command is not saturating //
     {
        intZ+= (rel_pos_2(2)+rel_pos_1(2))/20; //integral term in z, /20 because taking average of both measurements
        if(fabs(intZ*vecZ(2))>0.6){intZ=copysign(0.6/vecZ(2),intZ);}

      }
ROS_INFO("IntZ=%f",intZ);
//compute prop and deriv terms seperatly
wrench_1.header.stamp=ros::Time::now();
wrench_2.header.stamp=ros::Time::now();
wrench_1.wrench.force.x=0.9*(.1828*rel_pos_1(0)+.1828*rel_pos_2(0));
wrench_2.wrench.force.x=0.9*(1.9785*rel_vel_1(0)+1.9785*rel_vel_2(0));
wrench_1.wrench.force.y=0.9*(.1463*rel_pos_1(1)+.1463*rel_pos_2(1));
wrench_2.wrench.force.y=0.9*(1.7485*rel_vel_1(1)+1.7485*rel_vel_2(1));
wrench_1.wrench.force.z=0.9*(0.2704*rel_pos_1(2)+0.2704*rel_pos_2(2));
wrench_2.wrench.force.z=0.9*(2.0500*rel_vel_1(2)+2.0500*rel_vel_2(2));
wrench_1.wrench.torque.z=0.9*(0.075*rel_pos_1(1)-0.075*rel_pos_2(1)+0.075*rel_ang_1(2));
wrench_2.wrench.torque.z=0.9*(1.05*rel_vel_1(1)-1.05*rel_vel_2(1)+1.05*rel_avel_1(2));

wrench_1=wrench_1;
wrench_1=wrench_2;


MaxPrct_node.publish(maxPrctThrust);
force(2)=force(2)+intZ*vecZ(2);
wrench1_pub.publish(wrench_1);
wrench2_pub.publish(wrench_2);

glidePS1.header.stamp=ros::Time::now();
glidePS2.header.stamp=ros::Time::now();
glidePS1.wrench=vects2wrench(glidepos1,glidevel1);
glidePS2.wrench=vects2wrench(glidevel1,glidevel2);

glide1_pub.publish(glidePS1);
glide2_pub.publish(glidePS2);
netwrench=vects2wrench(force,torque);
netwrench_pub.publish(netwrench);

 loop_rate.sleep();

 }


return 0;
}