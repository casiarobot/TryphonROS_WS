
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
#include <dynamic_reconfigure/server.h>
#include <controls/controlConfig.h>


#include "controls/Commands.h"
#include <Euler_utility.h>
#include "vects2geoMsgs.cpp"

double dsz=0;
double dsx=0;
 
double dsy=0;
double dszwant=1000;
double errorn=0;
double erroro=0;
double errorz=0;
double deriv=0;
double rz=0;
double rzo=0;
double rz0=0;
int rzpos=1; // 0 between -360 and 0; 1-> 0-360; 2->360-720
geometry_msgs::Wrench F, FOld1, FOld2;
geometry_msgs::Pose fPose;
geometry_msgs::PoseStamped desirPose;
geometry_msgs::TwistStamped fVel;

Eigen::Vector3d force, forceOld1, forceOld2, forceGlobF, forceGfOld1, forceGfOld2, torque, torqueOld1, torqueOld2;
Eigen::Vector3d posdesir, pos, dPos, dPosOld1, dPosOld2, angledesir, angle, dAngle, dAngleOld1, dAngleOld2;
Eigen::Vector3d posOrig, angleOrig;
Eigen::Vector3d vel, avel;
Eigen::Vector3d CMCpos(0,0,0);
Eigen::Vector3d CMIMUpos(0,0,0); // vector postion from Center of mass to IMU in tryphon frame
//Eigen::Vector3d CMCpos(0,0,0.18);
//Eigen::Vector3d CMIMUpos(1,0,-1.125); // vector postion from Center of mass to IMU in tryphon frame
Eigen::Matrix3d Rmatrix, CPCMIMUmatrix, RIMUmatrix,CPCMCmatrix;
Eigen::Quaterniond quatIMU(1, 0,0, 0); // quat of the rotation matrix between the tryphon frame and the IMU frame
Eigen::Quaterniond quatIMU2(1, 0,0, 0);
//Eigen::Quaterniond quatIMU(0.99255, 0,0.12187, 0); // quat of the rotation matrix between the tryphon frame and the IMU frame
//Eigen::Quaterniond quatIMU2(0.99255, 0,-0.12187, 0);


Eigen::Vector3d veldesir(0,0,0);
Eigen::Vector3d aveldesir(0,0,0);
Eigen::Vector3d acceldesir(0,0,0);
Eigen::Vector3d angleAcceldesir(0,0,0);



bool start,On,path;
double GainCP, Cd;
double massTotal;
double gain = exp(-0.1/1); //gain = exp(-SamplingPeriod/FilterTimeConstant);
double fbuoyCoeff=1;       // gazebo parameter
int pathNb=0;              // Path nb wanted
int pathNbOld=0;              // Path nb wanted
int ctrlNb=0;              // Ctrl nb wanted
int GainFlip=1;

bool Command=false;     // getting the command from the website
bool startWeb=true;
bool noInt=false;          // increase integral term
std_msgs::Float64 maxPrctThrust;

void zero_vel()
{
  vect3_zero(veldesir);
  vect3_zero(aveldesir);
  vect3_zero(acceldesir);
  vect3_zero(angleAcceldesir);
}

Eigen::Matrix3d CPM(Eigen::Vector3d vect) // return cross product matrix
{
	Eigen::Matrix3d CPM;
	CPM<<       0,  -vect(2),   vect(1),
		  vect(2),         0,  -vect(0),
		  -vect(1),  vect(0),         0;

	return CPM;
}


void subStateT(const controls::State state)
{
  if(path)
  {
    posdesir=pose2vect_pos(state.pose);
    angledesir=pose2vect_angle(state.pose);
    veldesir=twist2vect_linear(state.vel);
    aveldesir=twist2vect_angular(state.vel);
    acceldesir=twist2vect_linear(state.accel);
    angleAcceldesir=twist2vect_angular(state.accel);
    ctrlNb=state.ctrlNb;
  }

}


void glideT(const controls::State gstate)
{
  {
    posdesir=pose2vect_pos(gstate.pose);
    angledesir=pose2vect_angle(gstate.pose);
    veldesir=twist2vect_linear(gstate.vel);
    aveldesir=twist2vect_angular(gstate.vel);
    acceldesir=twist2vect_linear(gstate.accel);
    angleAcceldesir=twist2vect_angular(gstate.accel);

    maxPrctThrust.data=gstate.maxThrust;
    GainCP=gstate.GainCP;
    noInt=gstate.noInt;

  }

} 


void subState(const state::state state)
{
  /*//Starting Pose needed only when the inertial frame is not centered//
  if(start==true){
    posOrig(0)=state.pos[0];
    posOrig(1)=state.pos[1];
    posOrig(2)=state.pos[2];
    quatorig(0)=state.quat[0];
    quatorig(1)=state.quat[1];
    quatorig(2)=state.quat[2];
    quatorig(3)=state.quat[3];
    velorig(0)=state.vel[0];
    velorig(1)=state.vel[1];
    velorig(2)=state.vel[2];
    avelorig(0)=state.angvel[0];
    avelorig(1)=state.angvel[1];
    avelorig(2)=state.angvel[2];
    thetaorig=atan2(2*(state.quat[0]*state.quat[3]+state.quat[1]*state.quat[2]),1-2*(state.quat[2]*state.quat[2]+state.quat[3]*state.quat[3]));
    ROS_INFO("Init pose done");
  }*/
  pos(0)=state.pos[0]; // define in global frame
  pos(1)=state.pos[1];
  pos(2)=state.pos[2];
  Eigen::Quaterniond quat(state.quat[0],state.quat[1],state.quat[2],state.quat[3]);
  vel(0)=state.vel[0]; // define in global frame
  vel(1)=state.vel[1];
  vel(2)=state.vel[2];
  avel(0)=state.angvel[0]; // define in body frame
  avel(1)=state.angvel[1];
  avel(2)=state.angvel[2];
  angle(0)=atan2(2*(quat.w()*quat.x()+quat.y()*quat.z()),1-2*(quat.x()*quat.x()+quat.y()*quat.y()));
  angle(1)=asin(2*(quat.w()*quat.y()-quat.z()*quat.x()));
  angle(2)=atan2(2*(quat.w()*quat.z()+quat.x()*quat.y()),1-2*(quat.z()*quat.z()+quat.y()*quat.y()));
  Rmatrix=quat.toRotationMatrix();
  start=false;

}

/*
void subPose_ekf(const geometry_msgs::PoseStamped PoseS)
{


  geometry_msgs::Pose Pose=PoseS.pose;
  if(start)
  {
    RIMUmatrix=quatIMU.toRotationMatrix(); // need to be computed only once
    CPCMIMUmatrix=CPM(CMIMUpos);
    CPCMCmatrix=CPM(CMCpos);
    start=false;
  }

  pos(0)=Pose.position.x; // defined in global frame
  pos(1)=Pose.position.y;
  pos(2)=Pose.position.z;
  Eigen::Quaterniond quat(Pose.orientation.w,Pose.orientation.x,Pose.orientation.y,Pose.orientation.z);
  Eigen::Quaterniond quat1(Pose.orientation.w,Pose.orientation.x,Pose.orientation.y,Pose.orientation.z);
  Eigen::Quaterniond quat2(Pose.orientation.w,Pose.orientation.x,Pose.orientation.y,Pose.orientation.z);
  Eigen::Quaterniond quat3(Pose.orientation.w,Pose.orientation.x,Pose.orientation.y,Pose.orientation.z);
  quat=quat*quatIMU.inverse();  // compute the quaternion between the vision world and the tryphon frame

  angle(0)=atan2(2*(quat.w()*quat.x()+quat.y()*quat.z()),1-2*(quat.x()*quat.x()+quat.y()*quat.y()));
  angle(1)=asin(2*(quat.w()*quat.y()-quat.z()*quat.x()));
  angle(2)=atan2(2*(quat.w()*quat.z()+quat.x()*quat.y()),1-2*(quat.z()*quat.z()+quat.y()*quat.y()));
  Rmatrix=quat.toRotationMatrix();

  pos=pos-Rmatrix*CMIMUpos;  // offset due to the fact that the pose is the one of the IMU

} */


void subPose(const geometry_msgs::PoseStamped PoseS)
{


  geometry_msgs::Pose Pose=PoseS.pose;
  if(start)
  {
    RIMUmatrix=quatIMU.toRotationMatrix(); // need to be computed only once
    CPCMIMUmatrix=CPM(CMIMUpos);
    CPCMCmatrix=CPM(CMCpos);
    start=false;
  }

  pos(0)=Pose.position.x; // defined in global frame
  pos(1)=Pose.position.y;
  pos(2)=Pose.position.z;
 // Eigen::Quaterniond quat(Pose.orientation.w,Pose.orientation.x,Pose.orientation.y,Pose.orientation.z);
  //Eigen::Quaterniond quat1(Pose.orientation.w,Pose.orientation.x,Pose.orientation.y,Pose.orientation.z);
 // Eigen::Quaterniond quat2(Pose.orientation.w,Pose.orientation.x,Pose.orientation.y,Pose.orientation.z);
 // Eigen::Quaterniond quat3(Pose.orientation.w,Pose.orientation.x,Pose.orientation.y,Pose.orientation.z);
 // quat=quat*quatIMU.inverse();  // compute the quaternion between the vision world and the tryphon frame

  angle(0)=Pose.orientation.x;
  angle(1)=Pose.orientation.y;
  angle(2)=Pose.orientation.z;

  Eigen::Quaterniond quat;
  EulerU::getQuatFromEuler(quat,angle(0),angle(1),angle(2));


  Rmatrix=quat.toRotationMatrix();

  //pos=pos-Rmatrix*CMIMUpos;  // offset due to the fact that the pose is the one of the IMU


}

void subVel(const geometry_msgs::TwistStamped Velocities)
{

  geometry_msgs::Twist Vel=Velocities.twist;
  if(!start)
  {
    vel(0)=Vel.linear.x; // defined in global frame
    vel(1)=Vel.linear.y;
    vel(2)=Vel.linear.z;
    avel(0)=Vel.angular.x; // defined in IMU frame
    avel(1)=Vel.angular.y;
    avel(2)=Vel.angular.z;
    avel=RIMUmatrix*avel;  // defined in body frame

   // vel=vel+Rmatrix*CPCMIMUmatrix*avel; // compute the vel of the center of mass

  }
}


void subCommands(const controls::Commands commands)
{

  Command=commands.commandOnOff;
  On=commands.onOff;

  // Defining the origin pose //
  if(Command && startWeb)
  {
    angleOrig=angle;
    startWeb=false;
  }
  if(!Command && !startWeb){startWeb=true;}

  // Update commands //
  if(Command)
  {
    
    path=commands.path;
    pathNb=commands.pathNb;
    maxPrctThrust.data=commands.maxThrust;
    GainCP=commands.GainCP;
    noInt=commands.noInt;

  }

  // Updating the desir pose if not doing a trajectory //
  geometry_msgs::Pose dPose=commands.deltaPose;

  if(Command && !path)
  { zero_vel();

	ctrlNb=commands.ctrlNb;
    if(dPose.position.x<=MAX_X && dPose.position.x>=MIN_X)
    {posdesir(0)=dPose.position.x;}

    if(dPose.position.y<=MAX_Y && dPose.position.y>=MIN_Y)
    {posdesir(1)=dPose.position.y;}

    if(dPose.position.z<=MAX_Z && dPose.position.z>=MIN_Z)
    {posdesir(2)=dPose.position.z;}

    if(dPose.orientation.x<=MAX_AX && dPose.orientation.x>=MIN_AX)
    {angledesir(0)=dPose.orientation.x;}

    if(dPose.orientation.y<=MAX_AY && dPose.orientation.y>=MIN_AY)
    {angledesir(1)=dPose.orientation.y;}

    if(dPose.orientation.z<=MAX_AZ && dPose.orientation.z>=MIN_AZ)
    {angledesir(2)=dPose.orientation.z;}

    noInt=commands.noInt;
    maxPrctThrust.data=commands.maxThrust;
    GainCP=commands.GainCP;
  }


  //ROS_INFO("Dx: %f, Dy: %maxPrctThrustf, Dz: %f,DTx: %f, DTy: %f, DTz: %f",des_state.pos[0], des_state.pos[1], des_state.pos[2], des_state.quat[1], des_state.quat[2], des_state.quat[0]);
}

void callback(controls::controlConfig &config, uint32_t level) {
    ROS_INFO("Reconfigure Request: x: %f, y: %f, z: %f, roll: %f, pitch: %f, yaw: %f  ",
             config.x,
             config.y,
             config.z,
             config.roll,
             config.pitch,
             config.yaw);
    posdesir(0)=config.x,
    posdesir(1)=config.y;
    posdesir(2)=config.z;
    angledesir(0)=config.roll;
    angledesir(1)=config.pitch;
    angledesir(2)=config.yaw;
    fbuoyCoeff=config.fbuoy;
    path=config.path;
    pathNb=config.pathNb-1;
    ctrlNb=config.ctrlNb;
    GainCP=config.gaincp;
    Cd=config.cd;
    massTotal=config.massTotal;
    On=config.onOff;
    maxPrctThrust.data=config.maxThrust;
    noInt=config.noInt;
}

void all_vects_zero()
{
  vect3_zero(force);
  vect3_zero(forceOld1);
  vect3_zero(forceOld2);
  vect3_zero(forceGlobF);
  vect3_zero(forceGfOld1);
  vect3_zero(forceGfOld2);
  vect3_zero(torque);
  vect3_zero(torqueOld1);
  vect3_zero(torqueOld2);
  vect3_zero(dPos);
  vect3_zero(dPosOld1);
  vect3_zero(dPosOld2);
  //vect3_zero(posdesir);  // to allow reconfigue to fix the desired pose
  vect3_zero(vel);
  vect3_zero(angle);
  vect3_zero(dAngle);
  vect3_zero(dAngleOld1);
  vect3_zero(dAngleOld2);
  vect3_zero(angledesir);
  vect3_zero(avel);

}

double saturation (double value, double up, double down)
{
  if(value>up)
  {return value=up;}
  if(value<down)
  {return value=down;}  // Computed Torque Matrices and scalar//
  return value;

}




inline float IIR(float old, float in, float gain)
{
  return old + (1.0-gain)*(in-old);
}


double smooth (double u1, double u2, double e, double e1, double e2)
{  // Computed Torque Matrices and scalar//
  return 1.16826*u1-0.42411820*u2+0.0639643*e+0.127929*e1+0.0639643*e2; // fc=1hz
  //return 1.56450*u1-0.6436623*  u2+0.0197896*e+0.03957916533*e1+0.0197896*e2; // fc=0.5hz

}

double modulo(double f) // not a modulo but dealing with the discontinuity around pi and -pi
{
  if(f>M_PI*(1+0.05)) // +0.05 to create an hysteresis
  {
   f=f-2*M_PI;
  }
  else
  {
    if(f<-M_PI*(1+0.05))
    {
      f=f+2*M_PI;
    }
  }
  return f;
}


ros::Publisher Controle_node;
ros::Publisher Desired_pose_node;
ros::Publisher Pose_node;
ros::Publisher Vel_node;
ros::Publisher Fbuoy_node;
ros::Publisher Path_node;
ros::Publisher Forcez_node;
ros::Publisher MaxPrct_node;
//ros::Publisher Controle_notfiltered_node;



void mySigintHandler(int sig)
{
  // Do some custom action.
  // For example, publish a stop message to some other nodes.
  F.force.x=0;F.force.y=0;F.force.z=0;F.torque.x=0;F.torque.y=0;F.torque.z=0;
  Controle_node.publish(F);
  Forcez_node.publish(F);
  ROS_INFO("fx: %f, fy: %f, fz: %f,Tx: %f, Ty: %f, Tz: %f",F.force.x, F.force.y, F.force.z, F.torque.x, F.torque.y, F.torque.z);
  // All the default sigint handler does is call shutdown()
  ros::shutdown();
}


/*const char* get_ip()
{
  int fd;
  struct ifreq ifr;
  char *ip = new char[100];

  fd = socket(AF_INET, SOCK_DGRAM, 0);

  /* I want to get an IPv4 IP address 
  ifr.ifr_addr.sa_family = AF_INET;

  /* I want IP address attached to "eth0"
  strncpy(ifr.ifr_name, "eth0", IFNAMSIZ-1);

  ioctl(fd, SIOCGIFADDR, &ifr);

  close(fd);

  /* display result 
  sprintf(ip,"%s", inet_ntoa(((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr));
  std::string s = ip;
  std::replace(s.begin(), s.end(), '.', '_');
  //ip=s.c_str();
  return s.c_str();
} */

int main(int argc, char **argv)
{

 //char rosname[100],ip[100];
  //sprintf(rosname,"control_%s",get_ip());
  std::string s ;

  ros::init(argc, argv, "control");
  ros::NodeHandle node;
  ros::NodeHandle nh1("~");
  signal(SIGINT, mySigintHandler);


  // Publishers //
  Controle_node = node.advertise<geometry_msgs::Wrench>("command_control",1);
  
  Forcez_node = node.advertise<geometry_msgs::Wrench>("intFz_control",1);
  //Controle_notfiltered_node = node.advertise<geometry_msgs::Wrench>("command_control_filtered",1);
  Desired_pose_node = node.advertise<geometry_msgs::PoseStamped>("desired_pose",1);
  Pose_node = node.advertise<geometry_msgs::Pose>("control/pose",1);
  Vel_node = node.advertise<geometry_msgs::TwistStamped>("control/vel",1);
  Path_node = node.advertise<geometry_msgs::Pose2D>("path_command",1);
  MaxPrct_node = node.advertise<std_msgs::Float64>("max_thrust",1);


  Fbuoy_node = node.advertise<geometry_msgs::Wrench>("fbuoy",1);



  // Subscribers //
  ros::Subscriber subS = node.subscribe("state", 1, subState);
  ros::Subscriber subSt = node.subscribe("state_trajectory", 1, subStateT);
  ros::Subscriber subglide = node.subscribe("glide_des_state", 1, glideT);
  ros::Subscriber subC = node.subscribe("commands", 1, subCommands);
 // ros::Subscriber subP = node.subscribe("ekf_node/pose", 1, subPose_ekf);
  //ros::Subscriber subV = node.subscribe("ekf_node/velocity", 1, subVel);
  ros::Subscriber subP = node.subscribe("state_estimator/pose", 1, subPose);
  ros::Subscriber subV = node.subscribe("state_estimator/vel", 1, subVel);

  // Dynamic Reconfigure //
  dynamic_reconfigure::Server<controls::controlConfig> server;
  dynamic_reconfigure::Server<controls::controlConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  // Loop rate //
  ros::Rate loop_rate(10);

  // Max percentage thrust //


  maxPrctThrust.data=100;


  // LQR Gain Matrices //
  Eigen::Matrix3d PropMatrixforce, DerivMatrixforce, PropMatrixtorque, DerivMatrixtorque;
  Eigen::Vector3d vecZ;
/*
  PropMatrixforce << 2.000, 0, 0,
  0, 2.000, 0,
  0, 0, 2.0692;

  DerivMatrixforce << 9.798, 0, 0,
  0, 9.798, 0,
  0, 0, 9.9591;

  PropMatrixtorque << 0.1716, 0, 0,
  0, 0.1716, 0,
  0, 0, 2.6458;

  DerivMatrixtorque << 3.5052, 0, 0,
  0, 3.5150, 0,
  0, 0, 9.5741;
*/

  PropMatrixforce << 2.000, 0, 0,
  0, 2.000, 0,
  0, 0, 2.0692;

  DerivMatrixforce << 13.1149, 0, 0,
  0, 13.1149, 0,
  0, 0, 13.1248;

  PropMatrixtorque << 0.1716, 0, 0,
  0, 0.1716, 0,
  0, 0, 2.000;

  DerivMatrixtorque << 3.5052, 0, 0,
  0, 3.5150, 0,
  0, 0, 10.1980;


  vecZ(0)=0;
  vecZ(1)=0;  
  vecZ(2)=0.0141;



  // Computed Torque Matrices and scalar//
  Eigen::Vector3d DragForce, DragTorque;
  Eigen::Matrix3d Identity3,  MassM, InertiaM, KpF, KpT, KvF, KvT, K, kpT, kvT;
  // define Inertia and Mass matrix

  double DragCoeffF=Cd*0.5*2.15*2.15*1.205;
  double DragCoeffT=3.45987;

  Identity3<< 1,  0,  0,
  0,  1,  0,
  0,   0, 1;

  kpT<< 1,  0,  0,
  0,  1,  0,
  0,   0, 1;

  kvT<< 1,  0,  0,
  0,  1,  0,
  0,   0, 1;

  MassM=massTotal*Identity3;

  InertiaM<< 15.4,  0,  0,
  0,  15.6,  0,
  0,   0, 16;



  KpF=(0.09*Identity3)*GainCP;
  KpT=(0.06*kpT)*GainCP;

  KvF=(0.55*Identity3)*GainCP;
  KvT=(0.55*kvT)*GainCP;

  // Drag coeffs //


  double GainCPOld=GainCP;
  double GainFlipOld=GainFlip;
  double CdOld=Cd;
  double massTotalOld=massTotal;


  // Initializing vectors and defining parameters//

  all_vects_zero();
  wrench_zero(F);
  wrench_zero(FOld1);
  wrench_zero(FOld2);


double x_start,y_start,z_start,tz_start;

    if (nh1.getParam("x", x_start))
    {
      ROS_INFO("Got param: %f", x_start );
    	posdesir(0)=x_start;
    }
    else
    {
      ROS_ERROR("Failed to get param 'x'");
      posdesir(0)=0.0;
    }

   if (nh1.getParam("y", y_start))
    {
      ROS_INFO("Got param: %f", y_start );
      posdesir(1)=y_start;
    }
    else
    {
      ROS_ERROR("Failed to get param 'y'");
      posdesir(1)=0.0;
    }
    if (nh1.getParam("z", z_start))
    {
      ROS_INFO("Got param: %f", z_start );
    	posdesir(2)=z_start;
    }
    else
    {
      ROS_ERROR("Failed to get param 'z'");
      posdesir(2)=0.0;
    }
    if (nh1.getParam("tz", tz_start))
    {
      ROS_INFO("Got param: %f", tz_start );
    	angledesir(2)=tz_start;
    }
    else
    {
      ROS_ERROR("Failed to get param 'tz'");
      angledesir(2)=0.00;
    }

  start=true;

  ctrlNb=1;

  double intZ = 0;
  geometry_msgs::Wrench FUnfiltered;

  double maxFxy=1.24;
  double minFxy=-0.64;
  double maxFz=2.48;
  double minFz=-1.28;
  double maxT=0.64;
  double minT=-0.64;

  Eigen::Vector3d dVel, dAvel;

  /////////////////////////


  // Force Z integral term //

  Eigen::Vector3d intFzGlbf, intFz;
  geometry_msgs::Wrench Fz;
  
  // Torque integral term //
  double intTX=0;
  double intTY=0;
  double intTZ=0; 
  
  Eigen::Vector3d intT;

  //////////////////////////

  while (ros::ok())
  {
    ros::spinOnce();
    if(On)
    {
    if(!start)
    {
	  if(!path){zero_vel();}
      

      /// Computing the errors ///

      // avoiding the discontunity due to the angle around pi and -pi //
      dAngle(0)=modulo(angle(0)-angledesir(0));
      dAngle(1)=modulo(angle(1)-angledesir(1));
      dAngle(2)=modulo(angle(2)-angledesir(2));
      dPos=pos-posdesir;
      dVel=vel-veldesir;
      dAvel=avel-aveldesir;

      ////////////////////////////
      
      // Integral terms //
      if(fabs(force(2))<2.4 && fabs(force(1))<1.2 && fabs(force(0))<1.2 && !noInt) // increasing only if the command is not saturating //
     {
        intZ+= (pos(2)-posdesir(2))/10;
        if(fabs(intZ*vecZ(2))>0.6){intZ=copysign(0.6/vecZ(2),intZ);}

      }
      
      if(fabs(torque(2))<2 && fabs(torque(1))<2 && fabs(torque(0))<2 && !noInt) // increasing only if the command is not saturating //
     {
        intTX+= (dAngle(0))/10*0.05;
        if(fabs(intTX)>2){intTX=copysign(2,intTX);}
        intTY+= (dAngle(1))/10*0.05;
        if(fabs(intTY)>2){intTY=copysign(2,intTY);}
        intTZ=0;
        //intTZ+= (dAngle(2))/10*0.03;
        //if(fabs(intTZ)>2){intTZ=copysign(2,intTZ);}
		intT<<intTX,intTY,intTZ;
      }
      
      //////////////////////////////

      /// Updating the gain of the Computed torque ///
      if(GainCPOld!=GainCP || CdOld!=Cd || massTotalOld!=massTotal || GainFlipOld!=GainFlip)
      {
        DragCoeffF=Cd*0.5*2.15*2.15*1.205;

        MassM=massTotal*Identity3;

        KpF=(0.09*Identity3)*GainCP;
        KpT=(0.06*kpT)*GainCP;

        KvF=(0.55*Identity3)*GainCP;
        KvT=(0.55*kvT)*GainCP;

        GainCPOld=GainCP;
        CdOld=Cd;
        massTotalOld=massTotal;
        GainFlipOld=GainFlip;
      }


      ////////////////////////////////////
      ////       Controller           ////
      ////////////////////////////////////
      switch (ctrlNb)
      {
        case 1 :
              // LQR //
              forceGlobF=-( PropMatrixforce*(dPos) + DerivMatrixforce*(dVel)); // + intZ*vecZ); // put into the intFz vector
              torque=-(PropMatrixtorque*(dAngle)+DerivMatrixtorque*(dAvel));
              vect3_zero(intFzGlbf);
              intFzGlbf=-intZ*vecZ;
              break;

        case 2 :
              // PID //

               forceGlobF(0)=0.1304*forceGfOld1(0)                        +184.0696*-dPos(0)  -181.2870*-dPosOld1(0);
               forceGlobF(1)=0.1304*forceGfOld1(1)                        +184.0696*-dPos(1)  -181.2870*-dPosOld1(1);
               forceGlobF(2)=1.1304*forceGfOld1(2) -0.1304*forceGfOld2(2) +184.0749*-dPos(2)  -365.3519*-dPosOld1(2)  +181.2863*-dPosOld2(2);

               torque(0)=10.2*-dAngle(0) -10.0*-dAngleOld1(0);
               torque(1)=10.2*-dAngle(1) -10.0*-dAngleOld1(1);
               torque(2)=153.0*-dAngle(2)  -150.0*-dAngleOld1(2);

               vect3_zero(intFzGlbf);
               break;

        case 3 :
              // Computed Torque //
              DragForce(0)=DragCoeffF*vel(0)*vel(0);
              DragForce(1)=DragCoeffF*vel(1)*vel(1);
              DragForce(2)=DragCoeffF*vel(2)*vel(2);

              DragTorque(0)=DragCoeffT*avel(0)*avel(0);
              DragTorque(1)=DragCoeffT*avel(1)*avel(1);
              DragTorque(2)=DragCoeffT*avel(2)*avel(2);

              DragTorque=DragTorque+CPCMCmatrix*Rmatrix.inverse()*DragForce;			  //CPM_CM may be already defined (Cross Product Matrix Center of mass)

              forceGlobF=MassM*(acceldesir-KpF*dPos-KvF*dVel)+DragForce;             // BE CAREFULL of the sign before Kp and Kv because of the definition of Dpos Dvel , etc
              torque=InertiaM*(angleAcceldesir-KpT*dAngle-KvT*dAvel)+DragTorque+CPM(avel)*InertiaM*avel; //- intT;

              // Addition of the g(q) reduced to the difference of buoyancy and gravity

              //forceGlobF(2) = forceGlobF(2);// - 0.0141*intZ;  // put into the intFz vector
              vect3_zero(intFzGlbf);
              intFzGlbf(2)=-0.0141*intZ;
              break;
              
        case 4 :
			  // Computed Torque //
              DragForce(0)=DragCoeffF*vel(0)*vel(0);
              DragForce(1)=DragCoeffF*vel(1)*vel(1);
              DragForce(2)=DragCoeffF*vel(2)*vel(2);

              DragTorque(0)=DragCoeffT*avel(0)*avel(0);
              DragTorque(1)=DragCoeffT*avel(1)*avel(1);
              DragTorque(2)=DragCoeffT*avel(2)*avel(2);

              DragTorque=DragTorque+CPCMCmatrix*Rmatrix.inverse()*DragForce;			  //CPM_CM may be already defined (Cross Product Matrix Center of mass)

              forceGlobF=MassM*(acceldesir-KpF*dPos-KvF*dVel)+DragForce;   
              //forceGlobF(2) = forceGlobF(2);// - 0.0141*intZ;  // put into the intFz vector
              vect3_zero(intFzGlbf);
              intFzGlbf(2)=-0.0141*intZ;
              torque=angleAcceldesir;
              intTX=0;
              intTY=0;
              intTZ=0;
              
              break;


        default :

               vect3_zero(forceGlobF);
               vect3_zero(torque);
               break;
      }


      ////////////////////////////////////


      // Rotate Force into body-frame //
      force=Rmatrix.inverse()*forceGlobF;
      intFz=Rmatrix.inverse()*intFzGlbf;


      // Command //
      F.force.x=force(0);
      F.force.y=force(1);
      F.force.z=force(2);
      F.torque.x=torque(0);
      F.torque.y=torque(1);
      F.torque.z=torque(2);

      // Integral fz term //

      Fz.force.x=intFz(0);
      Fz.force.y=intFz(1);
      Fz.force.z=intFz(2);
      Fz.torque.x=0;
      Fz.torque.y=0;
      Fz.torque.z=0;

      // Update //
      forceGfOld2=forceGfOld1;
      torqueOld2=torqueOld1;

      forceGfOld1=forceGlobF;
      torqueOld1=torque;

      forceOld2=forceOld1;
      forceOld1=force;

      dPosOld2=dPosOld1;
      dPosOld1=dPos;

      dAngleOld2=dAngleOld1;
      dAngleOld1=dAngle;

      FOld2=FOld1;
      FOld1=F;


      // Poses //
      fPose.position.x=pos(0);
      fPose.position.y=pos(1);
      fPose.position.z=pos(2);
      fPose.orientation.x=angle(0);
      fPose.orientation.y=angle(1);
      fPose.orientation.z=angle(2);

      desirPose.header.stamp=ros::Time::now();
      desirPose.pose.position.x=posdesir(0);
      desirPose.pose.position.y=posdesir(1);
      desirPose.pose.position.z=posdesir(2);
      desirPose.pose.orientation.x=angledesir(0);
      desirPose.pose.orientation.y=angledesir(1);
      desirPose.pose.orientation.z=angledesir(2);


      fVel.twist.linear.x=vel(0);
      fVel.twist.linear.y=vel(1);
      fVel.twist.linear.z=vel(2);
      fVel.twist.angular.x=avel(0);
      fVel.twist.angular.y=avel(1);
      fVel.twist.angular.z=avel(2);


      /////////////////////////////////

      Controle_node.publish(F);
      Forcez_node.publish(Fz);
      Pose_node.publish(fPose);
      Desired_pose_node.publish(desirPose);
      Vel_node.publish(fVel);
    }

      geometry_msgs::Wrench info;
      info.force.x=fbuoyCoeff;
      Fbuoy_node.publish(info);

      geometry_msgs::Pose2D path_command;
      path_command.x=pathNb;
      path_command.theta=path;
      Path_node.publish(path_command);
      MaxPrct_node.publish(maxPrctThrust);
		ROS_INFO("vel x: %f, y: %f ,z: %f ",veldesir(0),veldesir(1),veldesir(2));
		ROS_INFO("avel x: %f, y: %f ,z: %f ",aveldesir(0),aveldesir(1),aveldesir(2));
		ROS_INFO("Dangle x: %f, y: %f ,z: %f ",dAngle(0),dAngle(1),dAngle(2));

      loop_rate.sleep();
    }
    else
    {
      vect3_zero(force);
      vect3_zero(torque);
      F.force.x=force(0);
      F.force.y=force(1);
      F.force.z=force(2);
      F.torque.x=torque(0);
      F.torque.y=torque(1);
      F.torque.z=torque(2);
      intZ=0;
      Controle_node.publish(F);
      Forcez_node.publish(F);
      loop_rate.sleep();
    }
  }



  vect3_zero(force);
  vect3_zero(torque);
  F.force.x=force(0);
  F.force.y=force(1);
  F.force.z=force(2);
  F.torque.x=torque(0);
  F.torque.y=torque(1);
  F.torque.z=torque(2);
  Controle_node.publish(F);
  Forcez_node.publish(F);

  return 0;
}




