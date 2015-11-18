
#include "controls.h"
#include "ctr_fuzzy.h"

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



bool start,path;
bool On = false;
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



void subPose(const geometry_msgs::PoseStamped PoseS)
{


  geometry_msgs::Pose Pose=PoseS.pose;
  if(noInt)
  {
    posdesir(0)=Pose.position.x; // defined in global frame
    posdesir(1)=Pose.position.y;
    posdesir(2)=Pose.position.z;

    angledesir(0)=0;//Pose.orientation.x;
    angledesir(1)=0;//Pose.orientation.y;
    angledesir(2)=Pose.orientation.z;
  }

  pos(0)=Pose.position.x; // defined in global frame
  pos(1)=Pose.position.y;
  pos(2)=Pose.position.z;

  angle(0)=0;//Pose.orientation.x;
  angle(1)=0;//Pose.orientation.y;
  angle(2)=Pose.orientation.z;

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

void callback(controls::controlConfig &config, uint32_t level) {
    ROS_INFO("Reconfigure Request: x: %f, y: %f, z: %f, roll: %f, pitch: %f, yaw: %f  ",
             config.x,
             config.y,
             config.z,
             config.roll,
             config.pitch,
             config.yaw);
    //posdesir(0)=config.x,
    //posdesir(1)=config.y;
    //posdesir(2)=config.z;
    //angledesir(0)=config.roll;
    //angledesir(1)=config.pitch;
    //angledesir(2)=config.yaw;
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

float limitVel(double val, double limit)
{
    return copysign(fmin(fabs(val),limit), val);
}

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
  Controle_node = node.advertise<geometry_msgs::Wrench>("/192_168_10_243/command_control",1);

  Forcez_node = node.advertise<geometry_msgs::Wrench>("intFz_controlFuck",1);
  //Controle_notfiltered_node = node.advertise<geometry_msgs::Wrench>("command_control_filtered",1);
  Desired_pose_node = node.advertise<geometry_msgs::PoseStamped>("desired_poseFuck",1);
  Pose_node = node.advertise<geometry_msgs::Pose>("control/poseFuck",1);
  Vel_node = node.advertise<geometry_msgs::TwistStamped>("control/velFuck",1);
  Path_node = node.advertise<geometry_msgs::Pose2D>("path_commandFuck",1);
  MaxPrct_node = node.advertise<std_msgs::Float64>("max_thrustFuck",1);


  Fbuoy_node = node.advertise<geometry_msgs::Wrench>("fbuoy",1);
  // Dynamic Reconfigure //
  dynamic_reconfigure::Server<controls::controlConfig> server;
  dynamic_reconfigure::Server<controls::controlConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);


  // Subscribers //
  ros::Subscriber subP = node.subscribe("state_estimator/pose", 1, subPose);
  ros::Subscriber subV = node.subscribe("state_estimator/vel", 1, subVel);

  // Loop rate //
  ros::Rate loop_rate(10);

  // Max percentage thrust //


  maxPrctThrust.data=100;

  Eigen::Vector3d intFzGlbf, intFz;
  Eigen::Vector3d vecZ;
  Eigen::Vector3d dVel, dAvel;
  vecZ(0)=0;
  vecZ(1)=0;
  vecZ(2)=0.0141;
  GainCP = 1;


  // Initializing vectors and defining parameters//

  all_vects_zero();
  wrench_zero(F);
  wrench_zero(FOld1);
  wrench_zero(FOld2);

  start=true;
  double intZ = 0;
  noInt=true;
  On=false;

  geometry_msgs::Wrench Fz;


  //////////////////////////

  while (ros::ok())
  {
    ros::spinOnce();
    if(On)
    {
      /// Computing the errors ///

      // avoiding the discontunity due to the angle around pi and -pi //
      dAngle(0)=modulo(angle(0)-angledesir(0));
      dAngle(1)=modulo(angle(1)-angledesir(1));
      dAngle(2)=modulo(angle(2)-angledesir(2));
      dPos=pos-posdesir;
      dVel=vel-veldesir;
      dAvel=avel-aveldesir;

      // Integral terms //
      if(maxPrctThrust.data < 20.0){intZ = 0;}

      if(fabs(force(2))<2.4 && fabs(force(1))<1.2 && fabs(force(0))<1.2 && !noInt) // increasing only if the command is not saturating //
     {
        intZ+= (pos(2)-posdesir(2))/10;
        if(fabs(intZ*vecZ(2))>0.6){intZ=copysign(0.6/vecZ(2),intZ);}

      }


              // PID //

      forceGlobF(0)=-(0.8*dPos(0) + 3*(limitVel(dPos(0)-dPosOld1(0),1.0))/10.0);
      forceGlobF(1)=0;//0.1304*forceGfOld1(1)                        +184.0696*-dPos(1)  -181.2870*-dPosOld1(1);
      forceGlobF(2)=-(0.8*dPos(2) + 3*(limitVel(dPos(2)-dPosOld1(2),1.0))/10.0) +intZ*0.038;

      torque(0)=0;//10.2*-dAngle(0) -10.0*-dAngleOld1(0);
      torque(1)=0;//10.2*-dAngle(1) -10.0*-dAngleOld1(1);
      torque(2)=-(2*dAngle(2) + 4*(limitVel(dAngle(2)-dAngle(2),1.0))/10);



      ////////////////////////////////////


      // Rotate Force into body-frame //
      //force=Rmatrix.inverse()*forceGlobF;
      //intFz=Rmatrix.inverse()*intFzGlbf;

      force=forceGlobF;
      intFz=intFzGlbf;

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


      geometry_msgs::Wrench info;
      info.force.x=fbuoyCoeff;
      Fbuoy_node.publish(info);

      geometry_msgs::Pose2D path_command;
      path_command.x=pathNb;
      path_command.theta=path;
      Path_node.publish(path_command);
      MaxPrct_node.publish(maxPrctThrust);
                ROS_INFO("pos x: %f, y: %f ,z: %f ",pos(0),pos(1),pos(2));
                ROS_INFO("angle x: %f, y: %f ,z: %f ",angle(0),angle(1),angle(2));
                ROS_INFO("Dangle x: %f, y: %f ,z: %f ",dAngle(0),dAngle(1),dAngle(2));
                ROS_INFO("fx: %f, fy: %f, fz: %f,Tx: %f, Ty: %f, Tz: %f",F.force.x, F.force.y, F.force.z, F.torque.x, F.torque.y, F.torque.z);

      loop_rate.sleep();
    }
    else
    {
      /*vect3_zero(force);
      vect3_zero(torque);
      F.force.x=force(0);
      F.force.y=force(1);
      F.force.z=force(2);
      F.torque.x=torque(0);
      F.torque.y=torque(1);
      F.torque.z=torque(2);

      Controle_node.publish(F);
      Forcez_node.publish(F);*/
      intZ=0;
      loop_rate.sleep();
      start=true;
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





