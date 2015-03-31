#include "controls.h"
#include "ctr_fuzzy.h"
/*#define DEVICE_I2C      "/dev/i2c-3"
typedef enum {FALSE, TRUE}
BOOLEAN;
typedef unsigned char BYTE;
#define INTERACTION     0
*/


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

#include <dynamic_reconfigure/server.h>
#include <controls/controlConfig.h>


#include "controls/webCommands.h"

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
geometry_msgs::Pose fPose, desirPose, fVel;


double thetaorig=0;
double theta=0;
double thetaold[3]={0,0,0};
double thetadesir=0;
double thetaderiv=0;


Eigen::Vector3d force, forceOld1, forceOld2, forceGlobF, forceGfOld1, forceGfOld2, torque, torqueOld1, torqueOld2;
Eigen::Vector3d posdesir, pos, dPos, dPosOld1, dPosOld2, angledesir, angle, dAngle, dAngleOld1, dAngleOld2;
Eigen::Vector3d posOrig, angleOrig;
Eigen::Vector3d vel, avel;
Eigen::Vector3d CMCpos(0,0,0.18);
Eigen::Vector3d CMIMUpos(1,0,-1.125); // vector postion from Center of mass to IMU in tryphon frame
Eigen::Matrix3d Rmatrix, CPCMIMUmatrix, RIMUmatrix,CPCMCmatrix;
Eigen::Quaterniond quatIMU(0.99255, 0,0.12187, 0); // quat of the rotation matrix between the tryphon frame and the IMU frame
Eigen::Quaterniond quatIMU2(0.99255, 0,-0.12187, 0);

// Computed Torque Matrices and scalar//
Eigen::Vector3d DragForce, DragTorque;
Eigen::Matrix3d Identity3,  MassM, InertiaM, KpF, KpT, KvF, KvT;
// define Inertia and Mass matrix




// Drag coeffs //

double DragCoeffF;
double DragCoeffT;


bool start,path;
double GainCP, Cd;
double massTotal;
double gain = exp(-0.1/1); //gain = exp(-SamplingPeriod/FilterTimeConstant);
double fbuoyCoeff=1;       // gazebo parameter
int pathNb=0;              // Path nb wanted
int pathNbOld=0;              // Path nb wanted
int ctrlNb=0;              // Ctrl nb wanted

bool webCommand=false;     // getting the command from the website
bool startWeb=true;
bool noInt=false;          // increase integral term


Eigen::Matrix3d CPM(Eigen::Vector3d vect) // return cross product matrix
{
	Eigen::Matrix3d CPM;
	CPM<<       0,  -vect(2),   vect(1),
		  vect(2),         0,  -vect(0),
		  -vect(1),  vect(0),         0;

	return CPM;
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


}

void subVel(const geometry_msgs::TwistStamped VelS)
{

  geometry_msgs::Twist Vel=VelS.twist;
  if(!start)
  {
    vel(0)=Vel.linear.x; // defined in global frame
    vel(1)=Vel.linear.y;
    vel(2)=Vel.linear.z;
    avel(0)=Vel.angular.x; // defined in IMU frame
    avel(1)=Vel.angular.y;
    avel(2)=Vel.angular.z;
    avel=RIMUmatrix*avel;  // defined in body frame

    vel=vel+Rmatrix*CPCMIMUmatrix*avel; // compute the vel of the center of mass

  }
}


void subdP(const controls::webCommands commands)
{

  webCommand=commands.onOff;

  // Defining the origin pose //
  if(webCommand && startWeb)
  {
    posOrig=pos;
    angleOrig=angle;
    startWeb=false;
  }
  if(!webCommand && !startWeb){startWeb=true;}

  // Update commands //
  if(webCommand)
  {
    ctrlNb=commands.ctrlNb;
    path=commands.path;
    pathNb=commands.pathNb;

  }

  // Updating the desir pose if not doing a trajectory //
  geometry_msgs::Pose dPose=commands.deltaPose;

  if(webCommand && !path)
  {

    if(dPose.position.x+posOrig(0)<=MAX_X && dPose.position.x+posOrig(0)>=MIN_X)
    {posdesir(0)=posOrig(0)+dPose.position.x;}

    if(dPose.position.y+posOrig(1)<=MAX_Y && dPose.position.y+posOrig(1)>=MIN_Y)
    {posdesir(1)=posOrig(1)+dPose.position.y;}

    if(dPose.position.z+posOrig(2)<=MAX_Z && dPose.position.z+posOrig(2)>=MIN_Z)
    {posdesir(2)=posOrig(2)+dPose.position.z;}

    if(dPose.orientation.x+angleOrig(0)<=MAX_AX && dPose.orientation.x+angleOrig(0)>=MIN_AX)
    {angledesir(0)=posOrig(0)+dPose.orientation.x;}

    if(dPose.orientation.y+angleOrig(1)<=MAX_AY && dPose.orientation.y+angleOrig(1)>=MIN_AY)
    {angledesir(1)=posOrig(1)+dPose.orientation.y;}

    if(dPose.orientation.z+angleOrig(2)<=MAX_AZ && dPose.orientation.z+angleOrig(2)>=MIN_AZ)
    {angledesir(2)=posOrig(2)+dPose.orientation.z;}

  }


  //ROS_INFO("Dx: %f, Dy: %f, Dz: %f,DTx: %f, DTy: %f, DTz: %f",des_state.pos[0], des_state.pos[1], des_state.pos[2], des_state.quat[1], des_state.quat[2], des_state.quat[0]);
}

void callback(controls::controlConfig &config, uint32_t level) {
  if(!webCommand)
  {
    ROS_INFO("Reconfigure Request: x: %f, y: %f, z: %f, yaw: %f  ",
             config.x,
             config.y,
             config.z,
             config.yaw);
    posdesir(0)=config.x,
    posdesir(1)=config.y;
    posdesir(2)=config.z;
    angledesir(2)=config.yaw;
    fbuoyCoeff=config.fbuoy;
    path=config.path;
    pathNb=config.pathNb-1;
    ctrlNb=config.ctrlNb;
    GainCP=config.gaincp;
    Cd=config.cd;
    massTotal=config.massTotal;
  }
}


void wrench_zero(geometry_msgs::Wrench &w)
{
  w.force.x=0;
  w.force.y=0;
  w.force.z=0;
  w.torque.x=0;
  w.torque.y=0;
  w.torque.z=0;
}

void vect3_zero(Eigen::Vector3d &vect)
{
  vect(0)=0;
  vect(1)=0;
  vect(2)=0;
}

void vect4_zero(Eigen::Vector4d &vect)
{
  vect(0)=1;
  vect(1)=0;
  vect(2)=0;
  vect(3)=0;
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
  {return value=down;}
  return value;

}

double pPoly(double coef[3], double begin, double t)
{
  double dt=t-begin;
  return coef[0]*pow(dt,3)+coef[1]*pow(dt,4)+coef[2]*pow(dt,5);
}

double vPoly(double coef[3], double begin, double t)
{
  double dt=t-begin;
  return 3*coef[0]*pow(dt,2)+4*coef[1]*pow(dt,3)+5*coef[2]*pow(dt,4);
}

double aPoly(double coef[3], double begin, double t)
{
  double dt=t-begin;
  return 6*coef[0]*dt+12*coef[1]*pow(dt,2)+20*coef[2]*pow(dt,3);
}

void ComputeCoeffs(double (&c)[3], double dt, double dp)
{
  if(dt>0)
  {
  c[0]=10.0/pow(dt,3)*dp;
  c[1]=-15.0/pow(dt,4)*dp;
  c[2]=6.0/pow(dt,5)*dp;
  }
  else
  {
  c[0]=0;
  c[1]=0;
  c[2]=0;
  }

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


const char* get_ip()
{
  int fd;
  struct ifreq ifr;
  char *ip = new char[100];

  fd = socket(AF_INET, SOCK_DGRAM, 0);

  /* I want to get an IPv4 IP address */
  ifr.ifr_addr.sa_family = AF_INET;

  /* I want IP address attached to "eth0" */
  strncpy(ifr.ifr_name, "eth0", IFNAMSIZ-1);

  ioctl(fd, SIOCGIFADDR, &ifr);

  close(fd);

  /* display result */
  sprintf(ip,"%s", inet_ntoa(((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr));
  std::string s = ip;
  std::replace(s.begin(), s.end(), '.', '_');
  //ip=s.c_str();
  return s.c_str();
}

int main(int argc, char **argv)
{

  char rosname[100],ip[100];
  sprintf(rosname,"control_%s",get_ip());
  std::string s, temp_arg ;

  ros::init(argc, argv, rosname);
  ros::NodeHandle node;
  signal(SIGINT, mySigintHandler);
  if (argc==2)
  {
    ROS_INFO("TARGET IS: %s", argv[1]);
  }
  else
  {
    ROS_ERROR("Failed to get param 'target'");
    return 0;
  }

  temp_arg = argv[1];
  std::replace(temp_arg.begin(), temp_arg.end(), '.', '_');

  // Publishers //
  sprintf(rosname,"/%s/command_control",temp_arg.c_str());
  Controle_node = node.advertise<geometry_msgs::Wrench>(rosname,1);
  sprintf(rosname,"/%s/intFz_control",temp_arg.c_str());
  Forcez_node = node.advertise<geometry_msgs::Wrench>(rosname,1);
  //sprintf(rosname,"/%s/command_control_filtered",temp_arg.c_str());
  //Controle_notfiltered_node = node.advertise<geometry_msgs::Wrench>(rosname,1);
  sprintf(rosname,"/%s/desired_pose",temp_arg.c_str());
  Desired_pose_node = node.advertise<geometry_msgs::Pose>(rosname,1);
  sprintf(rosname,"/%s/pose",temp_arg.c_str());
  Pose_node = node.advertise<geometry_msgs::Pose>(rosname,1);
  sprintf(rosname,"/%s/velocity",temp_arg.c_str());
  Vel_node = node.advertise<geometry_msgs::Pose>(rosname,1);
  sprintf(rosname,"/%s/path_info",temp_arg.c_str());
  Path_node = node.advertise<geometry_msgs::Pose2D>(rosname,1);
  sprintf(rosname,"/%s/max_thrust",temp_arg.c_str());
  MaxPrct_node = node.advertise<std_msgs::Float64>(rosname,1);


  Fbuoy_node = node.advertise<geometry_msgs::Wrench>("/fbuoy",1);



  // Subscribers //
  sprintf(rosname,"/%s/state",temp_arg.c_str());
  ros::Subscriber subS = node.subscribe(rosname, 1, subState);
  //ros::Subscriber subdP = node.subscribe("/desired_deltapose", 1, subdP);
  ros::Subscriber subP = node.subscribe("/ekf_node/pose", 1, subPose);
  ros::Subscriber subV = node.subscribe("/ekf_node/velocity", 1, subVel);

  // Dynamic Reconfigure //
  dynamic_reconfigure::Server<controls::controlConfig> server;
  dynamic_reconfigure::Server<controls::controlConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  // Loop rate //
  ros::Rate loop_rate(10);

  // Max percentage thrust //

  std_msgs::Float64 maxPrctThrust;
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



  // Computed Torque //

  DragCoeffF=Cd*0.5*2.15*2.15*1.205;
  DragCoeffT=3.45987;

  Identity3<< 1,  0,  0,
  0,  1,  0,
  0,   0, 1;

  MassM=massTotal*Identity3;

  InertiaM<< 15.4,  0,  0,
  0,  15.6,  0,
  0,   0, 16;



  KpF=(0.06*Identity3)*GainCP;
  KpT=(0.06*Identity3)*GainCP;

  KvF=(0.42*Identity3)*GainCP;
  KvT=(0.49*Identity3)*GainCP;

  double GainCPOld=GainCP;
  double CdOld=Cd;
  double massTotalOld=massTotal;

  //[0.06*x;0.09*varphi]+[0.42*v;0.44*w]


  //// PATHS ////
  // Defining paths //

  bool path_debut=true;
  double path_debut_time=0;

  /*
  std::vector<Eigen::Vector4d> pathLine; // Gazebo paths
  Eigen::Vector4d p1(-4.5,0,0,0);
  Eigen::Vector4d p2(4.5,0,0,0);
  pathLine.push_back( p1);
  pathLine.push_back( p2);

  std::vector<Eigen::Vector4d> pathLine2;

  Eigen::Vector4d p12(4.5,0,0,M_PI);
  Eigen::Vector4d p22(-4.5,0,0,M_PI);

  pathLine2.push_back(p1);
  pathLine2.push_back(p2);
  pathLine2.push_back(p12);
  pathLine2.push_back(p22);

  std::vector<Eigen::Vector4d> pathSquare;
  Eigen::Vector4d pS(-3.5,-3.5,0,0);
  pathSquare.push_back(pS);

  pS(0)=3.5;
  pathSquare.push_back(pS);

  pS(1)=3.5;
  pathSquare.push_back(pS);

  pS(0)=-3.5;
  pathSquare.push_back(pS);*/

  std::vector<Eigen::Vector4d> pathLine;
  Eigen::Vector4d p1(-1.5,-1.5,2.5,M_PI/4);
  Eigen::Vector4d p2(1.5,1.5,2.5,M_PI/4);
  pathLine.push_back( p1);
  pathLine.push_back( p2);

  std::vector<Eigen::Vector4d> pathLine2;

  Eigen::Vector4d p12(1.5,1.5,2.5,-M_PI*3/4);
  Eigen::Vector4d p22(-1.5,-1.5,2.5,-M_PI*3/4);

  pathLine2.push_back(p1);
  pathLine2.push_back(p2);
  pathLine2.push_back(p12);
  pathLine2.push_back(p22);

  std::vector<Eigen::Vector4d> pathSquare;
  Eigen::Vector4d pS(-2.5,-2.5,2.5,0);
  pathSquare.push_back(pS);

  pS(0)=2.5;
  pathSquare.push_back(pS);

  pS(1)=2.5;
  pathSquare.push_back(pS);

  pS(0)=-2.5;
  pathSquare.push_back(pS);

  std::vector<Eigen::Vector4d> pathSquare2;


  std::vector< std::vector<Eigen::Vector4d> > paths;

  paths.push_back(pathLine);
  paths.push_back(pathLine2);
  paths.push_back(pathSquare);
  //paths.push_back(pathSquare2);

  // Rotation parameters //
  double omega=0.0333333333333333333333333333333;
  double r=2.5;
  double r1=2;
  double r2=0.5;
  double t=0;


  //////////////////////

  /// Definition of the coeffs for the specials paths ///
  double coeffs1[3]={0,0,0};
  double coeffs2[3]={0,0,0};
  double coeffs3[3]={0,0,0};
  double coeffs4[3]={0,0,0};
  double coeffs5[3]={0,0,0};
  double coeffs6[3]={0,0,0};
  double coeffs7[3]={0,0,0};
  double coeffs8x[3]={0,0,0};
  double coeffs8y[3]={0,0,0};
  double coeffs9[3]={0,0,0};
  double coeffs10[3]={0,0,0};
  double coeffs11[3]={0,0,0};

  ComputeCoeffs(coeffs1,30.00,6.0);
  ComputeCoeffs(coeffs2,16.0,1.6);
  ComputeCoeffs(coeffs3,15.0,2.0);
  ComputeCoeffs(coeffs4,20.0,1.6);
  ComputeCoeffs(coeffs5,20.0,1.0/4.0*M_PI);
  ComputeCoeffs(coeffs6,30.0,2.5);
  ComputeCoeffs(coeffs7,30.0,1.6);
  ComputeCoeffs(coeffs8x,60.0,6.5);
  ComputeCoeffs(coeffs8y,60.0,6.0);
  ComputeCoeffs(coeffs9,60.0,M_PI);
  ComputeCoeffs(coeffs10,25.0,2.5);
  ComputeCoeffs(coeffs11,25.0,3.0/4.0*M_PI);


  // ros::Subscriber subA = node.subscribe("/android/imu", 1, poseCallback);

  // Initializing vectors and defining parameters//

  all_vects_zero();
  wrench_zero(F);
  wrench_zero(FOld1);
  wrench_zero(FOld2);


  start=true;
  path=false;

  ctrlNb=1;

  int step=0;
  double intZ = 0;
  geometry_msgs::Wrench FUnfiltered;
  Eigen::Vector4d range(0.15,0.15,0.20,0.1);
  Eigen::Vector4d rangeV(0.2,0.2,0.2,0.2);

  double maxFxy=1.24;
  double minFxy=-0.64;
  double maxFz=2.48;
  double minFz=-1.28;
  double maxT=0.64;
  double minT=-0.64;

  Eigen::Vector3d dVel, dAvel;

  Eigen::Vector3d veldesir(0,0,0);
  Eigen::Vector3d aveldesir(0,0,0);
  Eigen::Vector3d acceldesir(0,0,0);
  Eigen::Vector3d angleAcceldesir(0,0,0);

  /////////////////////////


  // Force Z integral term //

  Eigen::Vector3d intFzGlbf, intFz;
  geometry_msgs::Wrench Fz;


  //////////////////////////

  while (ros::ok())
  {
    ros::spinOnce();
    if(!start)
    {
      // Integral term //
      if(fabs(force(2))<1.2 && fabs(force(1))<0.6 && fabs(force(0))<0.6 && !noInt) // increasing only if the command is not saturating //
     {
        intZ+= (pos(2)-posdesir(2))/10;
        if(fabs(intZ*vecZ(2))>0.6){intZ=copysign(0.6/vecZ(2),intZ);}

      }

      /// Computing the errors ///

      // avoiding the discontunity due to the angle around pi and -pi //
      dAngle(0)=modulo(angle(0)-angledesir(0));
      dAngle(1)=modulo(angle(1)-angledesir(1));
      dAngle(2)=modulo(angle(2)-angledesir(2));
      dPos=pos-posdesir;
      dVel=vel-veldesir;
      dAvel=avel-aveldesir;

      ////////////////////////////

      /// Updating the gain of the Computed torque ///
      if(GainCPOld!=GainCP || CdOld!=Cd || massTotalOld!=massTotal)
      {
        DragCoeffF=Cd*0.5*2.15*2.15*1.205;

        MassM=massTotal*Identity3;

        KpF=(0.06*Identity3)*GainCP;
        KpT=(0.06*Identity3)*GainCP;

        KvF=(0.42*Identity3)*GainCP;
        KvT=(0.49*Identity3)*GainCP;

        GainCPOld=GainCP;
        CdOld=Cd;
        massTotalOld=massTotal;
      }



      // Path //
      if(path)
      {
        if(path_debut)
        {
          path_debut_time=ros::Time::now().toSec();
          path_debut=false;
        }
        /*if(pathNb!=pathNbOld)  // not working properly
        {

          pathNbOld=pathNb;
        }*/
        if(pathNb<3)
        {
          if(fabs(pos(0)-posdesir(0))<range(0) && fabs(pos(1)-posdesir(1))<range(1) && fabs(pos(2)-posdesir(2))<range(2) && fabs(dAngle(2))<range(3) && fabs(vel(0))<rangeV(0) && fabs(vel(1))<rangeV(1) && fabs(vel(2))<rangeV(2) && fabs(avel(2))<rangeV(3))
          {
            if(step<paths[pathNb].size()-1){step++;}
            else{step=0;}

          }
          posdesir(0)=paths[pathNb][step](0);
          posdesir(1)=paths[pathNb][step](1);
          posdesir(2)=paths[pathNb][step](2);
          angledesir(2)=paths[pathNb][step](3);
        }
        if(pathNb==3)
        {
          t= ros::Time::now().toSec() - path_debut_time;
          ROS_INFO("t= %f; omega= %f; sin= %f", t, omega,r*sin(omega*t) );


          posdesir(0)=r*sin(omega*t);
          posdesir(1)=r*cos(omega*t);
          posdesir(2)=2.5;

          veldesir(0)=omega*r*cos(omega*t);
          veldesir(1)=-omega*r*sin(omega*t);
          veldesir(2)=0;

          acceldesir(0)=-omega*omega*r*sin(omega*t);
          acceldesir(1)=-omega*omega*r*cos(omega*t);
          acceldesir(2)=0;
        }
        if(pathNb==4)
        {
          t= ros::Time::now().toSec() - path_debut_time;
          ROS_INFO("t= %f; omega= %f; sin= %f", t, omega,r*sin(omega*t) );


          /*posdesir(0)=r*sin(omega*t);
          posdesir(1)=r*cos(omega*t);
          posdesir(2)=2.5+r2*sin(omega*t);

          veldesir(0)=omega*r*cos(omega*t);
          veldesir(1)=-omega*r*sin(omega*t);
          veldesir(2)=omega*r2*cos(omega*t);

          acceldesir(0)=-omega*omega*r*sin(omega*t);
          acceldesir(1)=-omega*omega*r*cos(omega*t);
          acceldesir(2)=-omega*omega*r2*sin(omega*t);*/

          double value=0.7*t;

          while(value>M_PI)
          {
           value-=2*M_PI;
          }

          angledesir(2)=value;

        }
        if(pathNb==5)  // Attack (petit nom)
        {
          t= ros::Time::now().toSec() - path_debut_time;
          if(t<30) // go back
          {
            noInt=true;
            posdesir(0)=-pPoly(coeffs1,0,t);
            posdesir(1)=0;
            posdesir(2)=2.5;

            veldesir(0)=-vPoly(coeffs1,0,t);
            veldesir(1)=0;
            veldesir(2)=0;

            acceldesir(0)=-aPoly(coeffs1,0,t);
            acceldesir(1)=0;
            acceldesir(2)=0;
          }

          if(t>30 && t<40 ) // wait
          {
            vect3_zero(acceldesir);
            vect3_zero(angleAcceldesir);
            vect3_zero(veldesir);
            vect3_zero(aveldesir);
          }
          if(t>40 && t<56 ) // go up
          {
            posdesir(0)=-6;
            posdesir(1)=0;
            posdesir(2)=2.5+pPoly(coeffs2,40,t);

            veldesir(0)=0;
            veldesir(1)=0;
            veldesir(2)=vPoly(coeffs2,40,t);

            acceldesir(0)=0;
            acceldesir(1)=0;
            acceldesir(2)=aPoly(coeffs2,40,t);
          }

          if(t>56 && t<61 ) // wait
          {
            posdesir(0)=-6;
            posdesir(1)=0;
            posdesir(2)=4.1;

            vect3_zero(acceldesir);
            vect3_zero(angleAcceldesir);
            vect3_zero(veldesir);
            vect3_zero(aveldesir);
          }
          if(t>61 && t<101 ) // go down full speed and wait
          {
            posdesir(0)=2;
            posdesir(1)=0;
            posdesir(2)=2.5;
          }

          if(t>101 && t<116 ) // go back
          {
            noInt=false;
            posdesir(0)=2-pPoly(coeffs3,101,t);
            posdesir(1)=0;
            posdesir(2)=2.5;

            veldesir(0)=-vPoly(coeffs3,101,t);
            veldesir(1)=0;
            veldesir(2)=0;

            acceldesir(0)=-aPoly(coeffs3,101,t);
            acceldesir(1)=0;
            acceldesir(2)=0;
          }
          if(t>116 && t<136 ) // wait
          {
            vect3_zero(acceldesir);
            vect3_zero(angleAcceldesir);
            vect3_zero(veldesir);
            vect3_zero(aveldesir);
          }
          if(t>136)
          {
            pathNb+=1;
            path_debut_time=ros::Time::now().toSec();
          }



        }

        if(pathNb==6) // observator
        {
          t= ros::Time::now().toSec() - path_debut_time;
          if(t<36) // Three turns 3*2*M_PI/1
          {
            noInt=true;
            double value=0.348*t;

            while(value>M_PI)
            {
              value-=2*M_PI;
            }

            angledesir(2)=value;
            aveldesir(2)=0.348;

          }
          if(t>36 && t<38 ) // slow down
          {
            angledesir(2)=0;
            aveldesir(2)=0;
          }

          if(t>38 && t<58 ) // go up and turn
          {

            posdesir(0)=0;
            posdesir(1)=0;
            posdesir(2)=2.5+pPoly(coeffs4,38,t);

            veldesir(0)=0;
            veldesir(1)=0;
            veldesir(2)=vPoly(coeffs4,38,t);

            acceldesir(0)=0;
            acceldesir(1)=0;
            acceldesir(2)=aPoly(coeffs4,38,t);


            angledesir(2)=0.55*sin(0.17*(t-38));

            aveldesir(2)=0.17*0.55*cos(0.17*(t-38));

            angleAcceldesir(2)=-0.17*0.17*0.55*sin(12*(t-38));
          }
          if(t>58 && t<118) // turn -30 +30
          {
            posdesir(0)=0;
            posdesir(1)=0;
            posdesir(2)=4.1;

            veldesir(0)=0;
            veldesir(1)=0;
            veldesir(2)=0;

            angledesir(2)=0.55*sin(0.17*(t-38));

            aveldesir(2)=0.17*0.55*cos(0.17*(t-38));

            angleAcceldesir(2)=-0.17*0.17*0.55*sin(0.17*(t-38));
          }
          if(t>118 && t<138 ) // wait
          {
            noInt=false;
            angledesir(2)=0;
            vect3_zero(acceldesir);
            vect3_zero(angleAcceldesir);
            vect3_zero(veldesir);
            vect3_zero(aveldesir);
          }
          if(t>138)
          {
            pathNb+=1;
            path_debut_time=ros::Time::now().toSec();
          }

        }
        if(pathNb==7)
        {

          t= ros::Time::now().toSec() - path_debut_time;
          if(t>0 && t<20 ) // go down and in the corner and turn -1/4 pi
          {
            noInt=true;

            angledesir(2)=-pPoly(coeffs5,0,t);

            aveldesir(2)=-vPoly(coeffs5,0,t);

            angleAcceldesir(2)=-aPoly(coeffs5,0,t);
          }
          if(t>20 && t<50 ) // go down and in the corner
          {
            posdesir(0)=pPoly(coeffs6,20,t);
            posdesir(1)=pPoly(coeffs6,20,t);
            posdesir(2)=4.1-pPoly(coeffs7,20,t);

            veldesir(0)=vPoly(coeffs6,20,t);
            veldesir(1)=vPoly(coeffs6,20,t);
            veldesir(2)=-vPoly(coeffs7,20,t);

            acceldesir(0)=aPoly(coeffs6,20,t);
            acceldesir(1)=aPoly(coeffs6,20,t);
            acceldesir(2)=-aPoly(coeffs7,20,t);

            angledesir(2)=-1.0/4.0*M_PI;

            aveldesir(2)=0;

            angleAcceldesir(2)=0;
          }
          if(t>50 && t<57 ) // wait
          {
            posdesir(0)=2.5;
            posdesir(1)=2.5;
            posdesir(2)=2.5;
            vect3_zero(acceldesir);
            vect3_zero(angleAcceldesir);
            vect3_zero(veldesir);
            vect3_zero(aveldesir);
          }
          if(t>57 && t<117 ) // diagonal + rotation 3pi/4
          {
            posdesir(0)=2.5-pPoly(coeffs8x,57,t);
            posdesir(1)=2.5-pPoly(coeffs8y,57,t);
            posdesir(2)=2.5;

            veldesir(0)=-vPoly(coeffs8x,57,t);
            veldesir(1)=-vPoly(coeffs8y,57,t);
            veldesir(2)=0;

            acceldesir(0)=-aPoly(coeffs8x,57,t);
            acceldesir(1)=-aPoly(coeffs8y,57,t);
            acceldesir(2)=0;


            angledesir(2)=-1.0/4.0*M_PI+pPoly(coeffs9,57,t);

            aveldesir(2)=vPoly(coeffs9,57,t);

            angleAcceldesir(2)=aPoly(coeffs9,57,t);
          }
          if(t>117 && t<127 ) // wait
          {
            posdesir(0)=-4;
            posdesir(1)=-3.5;
            posdesir(2)=2.5;
            angledesir(2)=3.0/4.0*M_PI;
            vect3_zero(acceldesir);
            vect3_zero(angleAcceldesir);
            vect3_zero(veldesir);
            vect3_zero(aveldesir);
          }
          if(t>127 && t<152) // go center + rotation pi
          {
            posdesir(0)=-4+pPoly(coeffs10,127,t);
            posdesir(1)=-3.5+pPoly(coeffs10,127,t);
            posdesir(2)=2.0;

            veldesir(0)=vPoly(coeffs10,127,t);
            veldesir(1)=vPoly(coeffs10,127,t);
            veldesir(2)=0;

            acceldesir(0)=aPoly(coeffs10,127,t);
            acceldesir(1)=aPoly(coeffs10,127,t);
            acceldesir(2)=0;


            angledesir(2)=3.0*M_PI/4.0-pPoly(coeffs11,127,t);

            aveldesir(2)=-vPoly(coeffs11,127,t);

            angleAcceldesir(2)=-aPoly(coeffs11,127,t);
          }
          if(t>152 && t<172 ) // wait
          {
            noInt=false;

            posdesir(0)=0;
            posdesir(1)=0;
            posdesir(2)=2.5;
            angledesir(2)=0;
            vect3_zero(acceldesir);
            vect3_zero(angleAcceldesir);
            vect3_zero(veldesir);
            vect3_zero(aveldesir);
          }
          if(t>167)
          {
            pathNb+=1;
            path_debut_time=ros::Time::now().toSec();
          }

        }
        if(pathNb==8)
        {
          t= ros::Time::now().toSec() - path_debut_time;
          if(t<25 ) // go to the beginning
          {
            posdesir(0)=0;
            posdesir(1)=2.5;
            posdesir(2)=2.5;
          }
          if(t>25 && t<253 )
          {
            GainCP=0.1;
            maxPrctThrust.data=30;

            posdesir(0)=r*sin(omega*(t-25));
            posdesir(1)=r*cos(omega*(t-25));
            posdesir(2)=2.5;

            veldesir(0)=omega*r*cos(omega*(t-25));
            veldesir(1)=-omega*r*sin(omega*(t-25));
            veldesir(2)=0;

            acceldesir(0)=-omega*omega*r*sin(omega*(t-25));
            acceldesir(1)=-omega*omega*r*cos(omega*(t-25));
            acceldesir(2)=0;
          }
          if(t>253 && t<263 )
          {
            posdesir(0)=2.5;
            posdesir(1)=0;
            posdesir(2)=2.5;
            vect3_zero(acceldesir);
            vect3_zero(angleAcceldesir);
            vect3_zero(veldesir);
            vect3_zero(aveldesir);
          }
          if(t>263 && t<310)
          {
            posdesir(0)=0;
            posdesir(1)=0;
            posdesir(2)=2.0;
          }
        }
        ROS_INFO("Path number %i",pathNb);
      }
      else
      {
        maxPrctThrust.data=100;
        step=0;
        path_debut=true;
        vect3_zero(acceldesir);
        vect3_zero(angleAcceldesir);
        vect3_zero(veldesir);
        vect3_zero(aveldesir);

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
              torque=InertiaM*(angleAcceldesir-KpT*dAngle-KvT*dAvel)+DragTorque+CPM(avel)*InertiaM*avel;

              // Addition of the g(q) reduced to the difference of buoyancy

              forceGlobF(2) = forceGlobF(2);// - 0.0141*intZ;  // put into the intFz vector
              vect3_zero(intFzGlbf);
              intFzGlbf(2)=-0.0141*intZ;
              break;


        default :

               vect3_zero(forceGlobF);
               vect3_zero(torque);
               break;
      }


      ////////////////////////////////////

      // Saturation //
      /*forceGlobF(0)=saturation(forceGlobF(0),maxFxy,-maxFxy);
      forceGlobF(1)=saturation(forceGlobF(1),maxFxy,-maxFxy);
      forceGlobF(2)=saturation(forceGlobF(2),maxFz,-maxFz);*/


      // Rotate Force into body-frame //
      force=Rmatrix.inverse()*forceGlobF;
      intFz=Rmatrix.inverse()*intFzGlbf;


      // Saturation //y
      /*force(0)=saturation(force(0),maxFxy,minFxy);
      force(1)=saturation(force(1),maxFxy,minFxy);
      force(2)=saturation(force(2),maxFz,minFz);
      torque(0)=saturation(torque(0),maxT,minT);
      torque(1)=saturation(torque(1),maxT,minT);
      torque(2)=saturation(torque(2),maxT,minT);*/


      //The first saturation is there to avoid the issue arising when the saturation function is applied the second time.


      // Command //
      F.force.x=force(0);  // -intFz(0);                               //smooth(FOld1.force.x,FOld2.force.x,force(0),forceOld1(0),forceOld2(0));
      F.force.y=force(1);  // -intFz(1);                               //smooth(FOld1.force.y,FOld2.force.y,force(1),forceOld1(1),forceOld2(1));
      F.force.z=force(2);  // -intFz(2);                               //smooth(FOld1.force.z,FOld2.force.z,force(2),forceOld1(2),forceOld2(2));
      F.torque.x=torque(0);                               //smooth(FOld1.torque.x,FOld2.torque.x,torque(0),torqueOld1(0),torqueOld2(0));
      F.torque.y=torque(1);                               //smooth(FOld1.torque.y,FOld2.torque.y,torque(1),torqueOld1(1),torqueOld2(1));
      F.torque.z=torque(2);                               //smooth(FOld1.torque.z,FOld2.torque.z,torque(2),torqueOld1(2),torqueOld2(2));

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

      desirPose.position.x=posdesir(0);
      desirPose.position.y=posdesir(1);
      desirPose.position.z=posdesir(2);
      desirPose.orientation.x=angledesir(0);
      desirPose.orientation.y=angledesir(1);
      desirPose.orientation.z=angledesir(2);

      fVel.position.x=vel(0);
      fVel.position.y=vel(1);
      fVel.position.z=vel(2);
      fVel.orientation.x=avel(0);
      fVel.orientation.y=avel(1);
      fVel.orientation.z=avel(2);


      /////////////////////////////////
      //ROS_INFO("fx: %f, fy: %f, fz: %f,Tx: %f, Ty: %f, Tz: %f ",F.force.x,F.force.y,F.force.z,F.torque.x,F.torque.y,F.torque.z);
      //ROS_INFO("x: %f, y: %f, z: %f,vx: %f, vy: %f, vz: %f ",pos(0),pos(1),pos(2),vel(0),vel(1),vel(2));
      //ROS_INFO("x: %f, y: %f, z: %f",posdesir(0),posdesir(1),posdesir(2));
      //ROS_INFO("ax: %f, ay: %f, az: %f,avx: %f, avy: %f, avz: %f ",angle(0),angle(1),angle(2),avel(0),avel(1),avel(2));


      Controle_node.publish(F);
      Forcez_node.publish(Fz);
      Pose_node.publish(fPose);
      Desired_pose_node.publish(desirPose);
      Vel_node.publish(fVel);

      /*FUnfiltered.force.x=force(0);
      FUnfiltered.force.y=force(1);
      FUnfiltered.force.z=force(2);
      FUnfiltered.torque.x=torque(0);
      FUnfiltered.torque.y=torque(1);
      FUnfiltered.torque.z=torque(2);
      Controle_notfiltered_node.publish(FUnfiltered);*/
    }

    geometry_msgs::Wrench info;
    info.force.x=fbuoyCoeff;
    Fbuoy_node.publish(info);

    geometry_msgs::Pose2D path_info;
    path_info.x=pathNb;
    path_info.y=step;
    path_info.theta=path;
    Path_node.publish(path_info);
    MaxPrct_node.publish(maxPrctThrust);


    loop_rate.sleep();
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

  return 0;
}




