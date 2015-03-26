
#include <iostream>
#include <ros/ros.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <net/if.h>
#include <arpa/inet.h>
#include <signal.h>
#include "std_msgs/Bool.h"
#include <dynamic_reconfigure/server.h>
#include <controls/controlConfig.h>


#include "controls/webCommands.h"

#define PI 3.14159265

ros::Publisher posed_chaser;
ros::Publisher posed_target;
ros::Publisher veld_chaser;
ros::Publisher veld_target;
ros::Publisher magnet;


//tryphon 244 has docking on x-direction, confirmed

geometry_msgs::Pose p_chaser,p_target,p_rel, pdes_chaser, pdes_target, p_target_initial,veldes_chaser,veldes_target;
Eigen::Matrix3d Rmatrix_chaser,CPCMIMUmatrix_chaser, RIMUmatrix_chaser,CPCMCmatrix_chaser, Rmatrix_target,CPCMIMUmatrix_target, RIMUmatrix_target,CPCMCmatrix_target;
Eigen::Vector3d pos_chaser,pos_target,angle_target,angle_chaser;
Eigen::Vector3d CMCpos(0,0,0.18);
Eigen::Vector3d CMIMUpos(1,0,-1.125);
Eigen::Quaterniond quatIMU(0.99255, 0,0.12187, 0); // quat of the rotation matrix between the tryphon frame and the IMU frame

double t=0;
double path_debut_time=0;
double sidelength_cube=2*1.125;
bool facing_each_other=false; //evaluates to true when line of sight guidance can begin for x_inital to be deteRmined
bool x_init_count=true; //ensures x init is set once
bool count_targ1=true;//this will ensure the orinal position of target is maintained for all info
std_msgs::Bool magnet_on;
bool start_T=true; //this will confiRm when target pose has been acquired at least once 
bool start_C=true;


Eigen::Matrix3d CPM(Eigen::Vector3d vect) // return cross product matrix
{
	Eigen::Matrix3d CPM;
	CPM<<       0,  -vect(2),   vect(1),
		  vect(2),         0,  -vect(0),
		  -vect(1),  vect(0),         0;

	return CPM;
}

void subPose_target(const geometry_msgs::PoseStamped PoseS1) //only need this for inital positions once facing each other
{

geometry_msgs::Pose Pose=PoseS1.pose;
  
  if(start_T)
  {
    RIMUmatrix_target=quatIMU.toRotationMatrix(); // need to be computed only once
    CPCMIMUmatrix_target=CPM(CMIMUpos);
    CPCMCmatrix_target=CPM(CMCpos);
    start_T=false;
  }

  pos_target(0)=Pose.position.x; // defined in global frame
  pos_target(1)=Pose.position.y;
  pos_target(2)=Pose.position.z;
  Eigen::Quaterniond quat(Pose.orientation.w,Pose.orientation.x,Pose.orientation.y,Pose.orientation.z);
 
  quat=quat*quatIMU.inverse();  // compute the quaternion between the vision world and the tryphon frame

  angle_target(0)=atan2(2*(quat.w()*quat.x()+quat.y()*quat.z()),1-2*(quat.x()*quat.x()+quat.y()*quat.y()));
  angle_target(1)=asin(2*(quat.w()*quat.y()-quat.z()*quat.x()));
  angle_target(2)=atan2(2*(quat.w()*quat.z()+quat.x()*quat.y()),1-2*(quat.z()*quat.z()+quat.y()*quat.y()));
  Rmatrix_target=quat.toRotationMatrix();

  pos_target=pos_target-Rmatrix_target*CMIMUpos;  // offset due to the fact that the pose is the one of the IMU, now at center of mass


p_target.position.x=pos_target(0);
p_target.position.y=pos_target(1);
p_target.position.z=pos_target(2);
p_target.orientation.x=angle_target(0); //remap inertial roll pitch yaw angles to the pose quaternions where w is not used
p_target.orientation.y=angle_target(1);
p_target.orientation.z=angle_target(2);
//keeps track that at least one pose has been called for target
}

void subPose_chaser(const geometry_msgs::PoseStamped PoseS2) 
{

geometry_msgs::Pose Pose=PoseS2.pose;
  
  if(start_C)
  {
    RIMUmatrix_chaser=quatIMU.toRotationMatrix(); // need to be computed only once
    CPCMIMUmatrix_chaser=CPM(CMIMUpos);
    CPCMCmatrix_chaser=CPM(CMCpos);
    start_C=false;
  }

  pos_chaser(0)=Pose.position.x; // defined in global frame
  pos_chaser(1)=Pose.position.y;
  pos_chaser(2)=Pose.position.z;
  Eigen::Quaterniond quat(Pose.orientation.w,Pose.orientation.x,Pose.orientation.y,Pose.orientation.z);
 
  quat=quat*quatIMU.inverse();  // compute the quaternion between the vision world and the tryphon frame

  angle_chaser(0)=atan2(2*(quat.w()*quat.x()+quat.y()*quat.z()),1-2*(quat.x()*quat.x()+quat.y()*quat.y()));
  angle_chaser(1)=asin(2*(quat.w()*quat.y()-quat.z()*quat.x()));
  angle_chaser(2)=atan2(2*(quat.w()*quat.z()+quat.x()*quat.y()),1-2*(quat.z()*quat.z()+quat.y()*quat.y()));
  Rmatrix_chaser=quat.toRotationMatrix();

  pos_chaser=pos_chaser-Rmatrix_chaser*CMIMUpos;  // offset due to the fact that the pose is the one of the IMU


p_chaser.position.x=pos_chaser(0);
p_chaser.position.y=pos_chaser(1);
p_chaser.position.z=pos_chaser(2);
p_chaser.orientation.x=angle_chaser(0);
p_chaser.orientation.y=angle_chaser(1);
p_chaser.orientation.z=angle_chaser(2);
}

void pose_zero(geometry_msgs::Pose &p) //set any pose msg to zero
{
	p.position.x=0;
	p.position.y=0;
	p.position.z=0;
	p.orientation.w=0; 
	p.orientation.x=0;
	p.orientation.y=0;
	p.orientation.z=0;
}



bool facing_error(const geometry_msgs::Pose Pose, double yaw) //returns true when the yaw desired (range) is achieved 
{

double yaw_error_allowed=0.05; //set yaw range
double angle_chaser_z;
//Eigen::Quaterniond quat(Pose.orientation.w,Pose.orientation.x,Pose.orientation.y,Pose.orientation.z);
angle_chaser_z=Pose.orientation.z;    ///atan2(2*(quat.w()*quat.z()+quat.x()*quat.y()),1-2*(quat.z()*quat.z()+quat.y()*quat.y()));

if (abs(angle_chaser_z-yaw)<=yaw_error_allowed)
	{
	return true;
	}
else {return false;}

}

int main(int argc, char **argv) //arguments should be IP of chaser then IP of target
{
//////ROS node setup with IP of chaser
std::string s; //temp_arg;
magnet_on.data=false;

char rosname[100],rosnameC[100],rosnameT[100],ip[100]; 
s=argv[1];
std::replace(s.begin(), s.end(), '.', '_');
sprintf(rosnameC,"%s",s.c_str());
s=argv[1];
std::replace(s.begin(), s.end(), '.', '_');
sprintf(rosnameT,"%s",s.c_str()); //used for target and chaser

ros::init(argc, argv,rosnameC);
ros::NodeHandle node;

if (argc==3)
  {
	    ROS_INFO("CHASER IS: %s", argv[1]);
	    ROS_INFO("TARGET IS: %s", argv[2]);
  }
else
  {
    ROS_ERROR("Failed to get param 'target' or 'chaser'");
    return 0;
  }

ros::Rate loop_rate(10); 

/*
temp_arg = argv[1];
 std::replace(temp_arg.begin(), temp_arg.end(), '.', '_');


if (argc==2)
  {
    ROS_INFO("TARGET IS: %s", argv[1]);
  }
else
  {
    ROS_ERROR("Failed to get param 'target'");
    return 0;
  }
ros::Rate loop_rate(10); 
//////End of ROS Node setup
temp_arg = argv[1];
  std::replace(temp_arg.begin(), temp_arg.end(), '.', '_');
*/
magnet = node.advertise<std_msgs::Bool>("/magnet_on",1);

sprintf(rosname,"/%s/glide_dpos",rosnameC);
posed_chaser = node.advertise<geometry_msgs::Pose>(rosname,1);
sprintf(rosname,"/%s/glide_dvel",rosnameC);
veld_chaser = node.advertise<geometry_msgs::Pose>(rosname,1);
sprintf(rosname,"/%s/glide_dpos",rosnameT);
posed_target = node.advertise<geometry_msgs::Pose>(rosname,1);
sprintf(rosname,"/%s/glide_dvel",rosnameT);
veld_target = node.advertise<geometry_msgs::Pose>(rosname,1);

sprintf(rosname,"/%s/ekf_node/pose",rosnameC);
ros::Subscriber subP_chase = node.subscribe(rosname, 1, subPose_chaser); //c
sprintf(rosname,"/%s/ekf_node/pose",rosnameT);
ros::Subscriber supP_targ = node.subscribe(rosname, 1 , subPose_target); //must change this accordingly


pose_zero(p_chaser);
pose_zero(p_target);
pose_zero(p_rel);
pose_zero(pdes_target);
pose_zero(pdes_chaser);
pose_zero(p_target_initial);
pose_zero(veldes_chaser);
pose_zero(veldes_target);

bool path_debut=true;
///////////glideslope parameters
double a; //glideslope <0, should probably be some function of initial distance since it controls the period
double x_initial; //should actually be once the tryphons are alligned
double v_max; //maximum allowed velocity upon docking
double v_init; 
double period; //The glideslope algorithm has a period (TBD)
///////////end of glideslope parameters

double yawd_target;
double yawd_chaser;
double r,r_dot;

while (ros::ok())
	{
	ros::spinOnce(); 
	
	if(!start_T && !start_C)
		{

		p_rel.position.x=p_target.position.x-p_chaser.position.x;
		p_rel.position.y=p_target.position.y-p_chaser.position.y;

		if(count_targ1) //count arg becomes false after first loop
			{
			yawd_chaser=atan2(p_rel.position.y,p_rel.position.x); //finds yaw desired, careful for 0,0, working in -PI to PI
			if(yawd_chaser<=0)
				{ yawd_target=PI-yawd_chaser;}
			else
				{ yawd_target=yawd_chaser-PI;} //finds the yawd of target, make sure >PI works out
		

		//set initial position where chaser and target will face each other
			p_target_initial=p_target;
			pdes_target.position.x=p_target.position.x;
			pdes_target.position.y=p_target.position.y;
			pdes_target.position.z=p_target.position.z;
			pdes_target.orientation.z=yawd_target;

			pdes_chaser.position.x=p_chaser.position.x;
			pdes_chaser.position.y=p_chaser.position.y;
			pdes_chaser.position.z=p_target.position.z; //chaser goes to height of target
			pdes_chaser.orientation.z=yawd_chaser;

			count_targ1=false;
			}

		if (facing_error(p_target,yawd_target) && facing_error(p_chaser,yawd_chaser)) //this deteRmines whether the two tryphons are close to facing each other
			{
			facing_each_other=true;//compare quaternion of the angle_chasers and makesure within a certain range of facing each other
			} 

		if (facing_each_other) //glideslope loop for which begins once tryphons are facing each other
			{	
		
			if (path_debut) // sets up the inital conditions of glideslope
				{
				
    			path_debut_time=ros::Time::now().toSec();
   
				x_initial=-sqrt(pow(p_rel.position.x,2.0)+pow(p_rel.position.y,2.0))+sidelength_cube; //noRm of the relative velocity vector
				a=(.01*x_initial); //define slope as a function of x_initial, (a distance of 4 m should take more time for docking than 3m)
				v_max=.02; //maximum allowed velocity upon docking
				v_init=a*x_initial+v_max; 
				period=1.0/a*log(v_max/v_init); //The glideslope algorithm has a period (TBD)
				ROS_INFO("Period=%f", period); //this must be possible with force of thrusters
				magnet_on.data=true;
				path_debut=false;
				}
		
			t= ros::Time::now().toSec() - path_debut_time; 

			//r is a negative number measured from the target and r_dot is positive
			r=x_initial*exp(a*t)+v_max/a*(exp(a*t)-1); //x_initial should be defined as distance from
			r_dot=a*r+v_max;

			//calulcate desired position and velocity of chaser
			if(abs(yawd_target)<PI/2) //Addition of r_target with r_x, r_y of glideslope decomposed using yaw of target
				{
				pdes_chaser.position.x=p_target_initial.position.x-r*cos(yawd_target);//will have to play around with this to make sure its proper
				pdes_chaser.position.y=p_target_initial.position.y-r*sin(yawd_target);
				veldes_chaser.position.x=r_dot*cos(yawd_chaser);
				veldes_chaser.position.y=r_dot*sin(yawd_chaser);
				}
			else
				{
				pdes_chaser.position.x=p_target_initial.position.x+r*sin(yawd_target);
				pdes_chaser.position.y=p_target_initial.position.y+r*cos(yawd_target);	
				veldes_chaser.position.x=r_dot*cos(yawd_target);
				veldes_chaser.position.y=r_dot*sin(yawd_target);
				}


			} //end of glideslope alg
	

	/*if(t>=period) //attempt at stopping the glideslope after period
	{
		pose_zero(veld_chaser);
	} */

	posed_target.publish(pdes_target); //what is sent has orientation in euler z=yaw
	posed_chaser.publish(pdes_chaser); //what is sent has orientation in euler z=yaw
	veld_chaser.publish(veldes_chaser);
	veld_target.publish(veldes_target);
	magnet.publish(magnet_on);

	if (t>=period){break;} //break the loop after period so the desired pos,vel doesn't keep changing
	loop_rate.sleep();
	}	
	}
return 0;
}
