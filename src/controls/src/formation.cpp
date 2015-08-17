#include "controls.h"
#include "ctr_fuzzy.h"

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

#include <dynamic_reconfigure/server.h>
#include <controls/formationConfig.h>

#include "controls/State.h"
#include "controls/Commands.h"
#include <Euler_utility.h>
#include "vects2geoMsgs.cpp"

#define PI 3.14159265
///should publish a force wrench to control which control takes and outputs to FD so control can always be running



ros::Publisher formation_chaser_pub;
ros::Publisher formation_target_pub;
ros::Publisher gaz_diff_pub;

bool start_T=true; //used to set up Rotation matrix initially for target
bool start_C=true;
double K_pos_T,K_pos_C, K_ang_T, K_ang_C, K_vel_T ,K_vel_C, K_angv_T, K_angv_C, K_des_C;
geometry_msgs::PoseStamped posegazC,posegazT, pose_gaz_diff;

Eigen::Vector3d pos_target, angle_target, vel_target, avel_target, svPT,svAT, spPT, spAT;//desired trajectories, accel,vel, and delta vel between C and T
Eigen::Vector3d qa_desPT,qv_desPT,qp_desPT, qa_desAT,qv_desAT,qp_desAT; //position and angle vectors
Eigen::Vector3d pos_chaser, angle_chaser, vel_chaser, avel_chaser; 
Eigen::Vector3d qa_desPC,qv_desPC, qp_desPC, qa_desAC,qv_desAC, qp_desAC, svPC, svAC, spPC, spAC, dqPp, dqPv, dqAp, dqAv,dqPa,dqAa ;
Eigen::Matrix3d Rmatrix_target, CPCMIMUmatrix_target, RIMUmatrix_target, CPCMCmatrix_target;
Eigen::Matrix3d Rmatrix_chaser, CPCMIMUmatrix_chaser, RIMUmatrix_chaser, CPCMCmatrix_chaser;
Eigen::Matrix3d KPPT, KPAT, KVPT, KVAT , KAPT , KAAT; //gains for formation control KPFT: -->Prop/Vel/Accel Force/Torque Target/Chaser
Eigen::Matrix3d KPPC, KPAC, KVPC, KVAC , KAPC , KAAC;
Eigen::Vector3d CMIMUpos(0,0,0); 
Eigen::Vector3d CMCpos(0,0,0.18);
Eigen::Quaterniond quatIMU(1, 0,0, 0);

controls::State formdesirT, formdesirC; //these will correspond to q_doubledot, s_dot and s for formation keeping
///////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////

Eigen::Matrix3d CPM(Eigen::Vector3d vect) // return cross product matrix
{
	Eigen::Matrix3d CPM;
	CPM<<       0,  -vect(2),   vect(1),
		  vect(2),         0,  -vect(0),
		  -vect(1),  vect(0),         0;

	return CPM;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////

void gazeboC (const geometry_msgs::PoseStamped PGC)
{

posegazC.pose=PGC.pose;

}


void gazeboT (const geometry_msgs::PoseStamped PGT)
{

posegazT.pose=PGT.pose;

}

///////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////

void subPose_target(const geometry_msgs::PoseStamped PoseS1) //only need this for inital positions once facing each other
{

geometry_msgs::PoseStamped PoseT;

PoseT.pose=PoseS1.pose;
  
if(start_T)
	{
	RIMUmatrix_target=quatIMU.toRotationMatrix(); // need to be computed only once
	CPCMIMUmatrix_target=CPM(CMIMUpos);
	CPCMCmatrix_target=CPM(CMCpos);
	start_T=false;
	}

pos_target(0)=PoseT.pose.position.x; // defined in global frame
pos_target(1)=PoseT.pose.position.y;
pos_target(2)=PoseT.pose.position.z;
angle_target(0)=PoseT.pose.orientation.x;
angle_target(1)=PoseT.pose.orientation.y;
angle_target(2)=PoseT.pose.orientation.z;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

void subPose_chaser(const geometry_msgs::PoseStamped PoseS2) 
{

geometry_msgs::PoseStamped PoseC;
PoseC.pose=PoseS2.pose;
  
if(start_C)
	{
    RIMUmatrix_chaser=quatIMU.toRotationMatrix(); // need to be computed only once
    CPCMIMUmatrix_chaser=CPM(CMIMUpos);
    CPCMCmatrix_chaser=CPM(CMCpos);
    start_C=false;
	}

pos_chaser(0)=PoseC.pose.position.x; // defined in global frame
pos_chaser(1)=PoseC.pose.position.y;
pos_chaser(2)=PoseC.pose.position.z;
angle_chaser(0)=PoseC.pose.orientation.x;
angle_chaser(1)=PoseC.pose.orientation.y;
angle_chaser(2)=PoseC.pose.orientation.z;

}

///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

void subVel_target(const geometry_msgs::TwistStamped TwistS1)
{

geometry_msgs::Twist TwistT=TwistS1.twist;

if(!start_T)
	{
	vel_target(0)=TwistT.linear.x; // defined in global frame
    vel_target(1)=TwistT.linear.y;
    vel_target(2)=TwistT.linear.z;
    avel_target(0)=TwistT.angular.x; // defined in IMU frame
    avel_target(1)=TwistT.angular.y;
    avel_target(2)=TwistT.angular.z;
    avel_target=RIMUmatrix_target*avel_target;  // defined in body frame
	}

}
///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

void subVel_chaser(const geometry_msgs::TwistStamped TwistS2)
{

geometry_msgs::Twist TwistC=TwistS2.twist;

if(!start_C)
	{
	vel_chaser(0)=TwistC.linear.x; // defined in global frame
    vel_chaser(1)=TwistC.linear.y;
    vel_chaser(2)=TwistC.linear.z;
    avel_chaser(0)=TwistC.angular.x; // defined in IMU frame
    avel_chaser(1)=TwistC.angular.y;
    avel_chaser(2)=TwistC.angular.z;
    avel_chaser=RIMUmatrix_chaser*avel_chaser;  // defined in body frame
	}

}


//////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

void callback(controls::formationConfig &config, uint32_t level) 
{

ROS_INFO("Reconfigure Request: K_pos_T: %f, K_vel_T: %f, K_ang_T: %f, K_angv_T: %f  ",
             config.K_pos_T,
             config.K_vel_T,
             config.K_ang_T,
             config.K_angv_T);
            
ROS_INFO("Reconfigure Request: K_pos_C: %f, K_vel_C: %f, K_ang_C: %f, K_angv_C: %f  ",
             config.K_pos_C,
             config.K_vel_C,
             config.K_ang_C,
             config.K_angv_C);

K_pos_T=config.K_pos_T;
K_vel_T=config.K_vel_T;
K_ang_T=config.K_ang_T;
K_angv_T=config.K_angv_T;
K_pos_C=config.K_pos_C;
K_vel_C=config.K_vel_C;
K_ang_C=config.K_ang_C;
K_angv_C=config.K_angv_C;
K_des_C=config.K_des_C;

dqPp(0)=config.delta_x;
dqPp(1)=config.delta_y;
dqPp(2)=config.delta_z;
dqAp(2)=config.delta_yaw;
} 


//////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

void traj_target_callback(const controls::State state)
{

qp_desPT=pose2vect_pos(state.pose);
qp_desAT=pose2vect_angle(state.pose);

qv_desPT=twist2vect_linear(state.vel);
qv_desAT=twist2vect_angular(state.vel);

qa_desPT=twist2vect_linear(state.accel);
qa_desAT=twist2vect_angular(state.accel);


qp_desPC=qp_desPT+dqPp;
qp_desAC=qp_desAT+dqAp;

qv_desPC=qv_desPT+dqPv;
qv_desAC=qv_desAT+dqAv;

qa_desPC=qa_desPT+dqPa;
qa_desAC=qa_desAT+dqAa;
}


//////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////


int main(int argc, char  **argv)
{
	
ros::init(argc, argv, "formation");
ros::NodeHandle node;



//////ROS node setup with IP of chaser
std::string s; //temp_arg;
char rosname[100],rosnameC[100],rosnameT[100],ip[100]; 
s=argv[1];
//std::replace(s.begin(), s.end(), '.', '_');
sprintf(rosnameC,"%s",s.c_str());
s=argv[2];
//std::replace(s.begin(), s.end(), '.', '_');
sprintf(rosnameT,"%s",s.c_str()); //used for target and chaser

if (argc==3)
  {
	    ROS_INFO("CHASER (F) IS: %s", argv[1]);
	    ROS_INFO("TARGET (L) IS: %s", argv[2]);
  }
else
  {
    ROS_ERROR("Failed to get param 'target' or 'chaser'");
    return 0;
  }


/////Subscribers/////
sprintf(rosname,"/%s/state_estimator/pose",rosnameC);
ros::Subscriber subP_chase = node.subscribe(rosname, 1, subPose_chaser); 
sprintf(rosname,"/%s/state_estimator/vel",rosnameC);
ros::Subscriber subV_chase = node.subscribe(rosname, 1, subVel_chaser); 
sprintf(rosname,"/%s/state_estimator/pose",rosnameT);
ros::Subscriber supP_targ = node.subscribe(rosname, 1 , subPose_target); 
sprintf(rosname,"/%s/state_estimator/vel",rosnameT);
ros::Subscriber supV_targ = node.subscribe(rosname, 1 , subVel_target);
sprintf(rosname,"/%s/state_trajectory",rosnameT); //only needs the trajectory for target 
ros::Subscriber traj_T = node.subscribe(rosname, 1 , traj_target_callback);
sprintf(rosname,"/%s/poseStamped_gazebo",rosnameC);
ros::Subscriber gazebo_C = node.subscribe(rosname, 1, gazeboC); 
sprintf(rosname,"/%s/poseStamped_gazebo",rosnameT); //only needs the trajectory for target 
ros::Subscriber gazebo_T= node.subscribe(rosname, 1 , gazeboT);



/////Publishers/////
sprintf(rosname,"/%s/formation_state",rosnameT);
formation_target_pub=node.advertise<controls::State>(rosname,1);
sprintf(rosname,"/%s/formation_state",rosnameC);
formation_chaser_pub=node.advertise<controls::State>(rosname,1);
gaz_diff_pub=node.advertise<geometry_msgs::PoseStamped>("gaz_diff",1);


dynamic_reconfigure::Server<controls::formationConfig> server;
dynamic_reconfigure::Server<controls::formationConfig>::CallbackType f;

f = boost::bind(&callback, _1, _2);
server.setCallback(f);


//Default formation values

dqPp(0)=3.0;
dqPp(1)=3.0;
dqPp(2)=0.0;
dqAp(0)=0.0;
dqAp(1)=0.0;
dqAp(2)=0.0;

dqPv(0)=0.0;
dqPv(1)=0.0;
dqPv(2)=0.0;
dqAv(0)=0.0;
dqAv(1)=0.0;
dqAv(2)=0.0;

dqPa(0)=0.0;
dqPa(1)=0.0;
dqPa(2)=0.0;
dqAa(0)=0.0;
dqAa(1)=0.0;
dqAa(2)=0.0;

qp_desPT(0)=-1;
qp_desPT(1)=-1;
qp_desPC(0)=qp_desPT(0)+dqPp(0);
qp_desPC(1)=qp_desPT(1)+dqPp(1);

// Loop rate //
ros::Rate loop_rate(10);

//Gain matrices//
///////////////////////////////////////////////////////////////////////////////////////////////////
KPPT<< 1,  0,  0,
  0,  1,  0,
  0,   0, 1;
KPPC<< 1,  0,  0,
  0,  1,  0,
  0,   0, 1; 
KPAT<< 1,  0,  0,
  0,  1,  0,
  0,   0, 1; 
KPAC<< 1,  0,  0,
  0,  1,  0,
  0,   0, 1;  
KVPT<< 1,  0,  0,
  0,  1,  0,
  0,   0, 1;
KVPC<< 1,  0,  0,
  0,  1,  0,
  0,   0, 1;
KVAT<< 1,  0,  0,
  0,  1,  0,
  0,   0, 1;
KVAC<< 1,  0,  0,
  0,  1,  0,
  0,   0, 1;
KAPT<< 1,  0,  0,
  0,  1,  0,
  0,   0, 1;
KAPC<< 1,  0,  0,
  0,  1,  0,
  0,   0, 1;
KAAT<< 1,  0,  0,
  0,  1,  0,
  0,   0, 1;
KAAC<< 1,  0,  0,
  0,  1,  0,
  0,   0, 1;

///////////////////////////////////////////////////////////////////////////////////////////////////

while (ros::ok())
	{
    ros::spinOnce();

	spPT=qp_desPT-K_pos_T*KPPT*(pos_target-(pos_chaser-dqPp));
	spAT=qp_desAT-K_ang_T*KPAT*(angle_target-(angle_chaser-dqAp));
	svPT=qv_desPT-K_vel_T*KVPT*(vel_target-(vel_chaser-dqPv));
	svAT=qv_desAT-K_angv_T*KVAT*(avel_target-(avel_chaser-dqAv));

	spPC=K_des_C*qp_desPC-K_pos_C*KPPC*(pos_chaser-(pos_target+dqPp));
	spAC=K_des_C*qp_desAC-K_ang_C*KPAC*(angle_chaser-(angle_target+dqAp));
	svPC=K_des_C*qv_desPC-K_vel_C*KVPC*(vel_chaser-(vel_target+dqPv));
	svAC=K_des_C*qv_desAC-K_angv_C*KVAC*(avel_chaser-(avel_target+dqAv));


	formdesirT.header.stamp=ros::Time::now();
	formdesirT.pose=vects2pose(spPT,spAT);
	formdesirT.vel=vects2twist(svPT,svAT);
	formdesirT.accel=vects2twist(qa_desPT,qa_desAT);

	formdesirC.header.stamp=ros::Time::now();
	formdesirC.pose=vects2pose(spPC,spAC);
	formdesirC.vel=vects2twist(svPC,svAC);
	formdesirC.accel=vects2twist(qa_desPC,qa_desAC);

////////////////////////////////////////////////////////////////////////////////////////////////////////

  pose_gaz_diff.header.stamp=ros::Time::now();
  pose_gaz_diff.pose.position.x=posegazT.pose.position.x-posegazC.pose.position.x;
  pose_gaz_diff.pose.position.y=posegazT.pose.position.y-posegazC.pose.position.y;


////////////////////////////////////////////////////////////////////////////////////////////////////////


	formation_target_pub.publish(formdesirT);
	formation_chaser_pub.publish(formdesirC);
  gaz_diff_pub.publish(pose_gaz_diff);

	loop_rate.sleep();	
	}




return 0;
}