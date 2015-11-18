// Node working with the ps3XZLC

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
#include "std_msgs/Int64.h"
#include <dynamic_reconfigure/server.h>
#include <controls/controlConfig.h>


#include "controls/Commands.h"
#include <Euler_utility.h>
#include "vects2geoMsgs.cpp"


geometry_msgs::Wrench F, FOld1, FOld2, ps3Wrench;
geometry_msgs::Pose fPose;
geometry_msgs::PoseStamped desirPose;
geometry_msgs::TwistStamped fVel;

Eigen::Vector3d force, forceOld1, forceGlobF, forceGfOld1, torque, torqueOld1;
Eigen::Vector3d posdesir, pos, posinit, dPos, dPosOld1, angledesir, angle, angleinit, dAngle, dAngleOld1;
Eigen::Vector3d posOrig, angleOrig;
Eigen::Vector3d vel, avel;
Eigen::Vector3d CMCpos(0,0,0);
Eigen::Vector3d CMIMUpos(0,0,0); // vector postion from Center of mass to IMU in tryphon frame
Eigen::Matrix3d Rmatrix, CPCMIMUmatrix, RIMUmatrix,CPCMCmatrix;
Eigen::Quaterniond quatIMU(1, 0,0, 0); // quat of the rotation matrix between the tryphon frame and the IMU frame
Eigen::Quaterniond quatIMU2(1, 0,0, 0);

Eigen::Vector3d veldesir(0,0,0);
Eigen::Vector3d aveldesir(0,0,0);
Eigen::Vector3d acceldesir(0,0,0);
Eigen::Vector3d angleAcceldesir(0,0,0);

double gain = exp(-0.1/1); //gain = exp(-SamplingPeriod/FilterTimeConstant);
int mode = 0;
double intZ;

bool resetInt = false;


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

  pos(0)=Pose.position.x; // defined in global frame
  pos(1)=0;//Pose.position.y;
  pos(2)=Pose.position.z;

  angle(0)=0;//Pose.orientation.x;
  angle(1)=0;//Pose.orientation.y;
  angle(2)=Pose.orientation.z;

}

void subVel(const geometry_msgs::TwistStamped Velocities)
{

  geometry_msgs::Twist Vel=Velocities.twist;
  vel(0)=Vel.linear.x; // defined in global frame
  vel(1)=Vel.linear.y;
  vel(2)=Vel.linear.z;
  avel(0)=Vel.angular.x; // defined in IMU frame
  avel(1)=Vel.angular.y;
  avel(2)=Vel.angular.z;
  avel=RIMUmatrix*avel;  // defined in body frame
}

void subDeltaPose(const geometry_msgs::Pose deltapose)
{
    posdesir = posinit + pose2vect_pos(deltapose);
    angledesir = angleinit + pose2vect_angle(deltapose);
}

void subPS3(const geometry_msgs::Wrench w)
{
    ps3Wrench = w;
}

void subMode(const std_msgs::Int64 md)
{
    int m = md.data;
    if(mode == 0 && m ==1)
    {
        mode = 1;

        posinit(0)=pos(0); // defined in global frame
        posinit(1)=0;
        posinit(2)=pos(2);

        angleinit(0)=0;//Pose.orientation.x;
        angleinit(1)=0;//Pose.orientation.y;
        angleinit(2)=angle(2);

        intZ=10;
    }
    if(mode == 1 && m ==0)
    {
        mode = 0;
    }
}


void all_vects_zero()
{
  vect3_zero(force);
  vect3_zero(forceOld1);
  vect3_zero(forceGlobF);
  vect3_zero(forceGfOld1);
  vect3_zero(torque);
  vect3_zero(torqueOld1);
  vect3_zero(dPos);
  vect3_zero(dPosOld1);
  //vect3_zero(posdesir);  // to allow reconfigue to fix the desired pose
  vect3_zero(vel);
  vect3_zero(angle);
  vect3_zero(dAngle);
  vect3_zero(dAngleOld1);
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


  // Subscribers //
  ros::Subscriber subP = node.subscribe("state_estimator/pose", 1, subPose);
  ros::Subscriber subV = node.subscribe("state_estimator/vel", 1, subVel);
  ros::Subscriber subDe = node.subscribe("desired_deltapose", 1, subDeltaPose);
  ros::Subscriber subPS = node.subscribe("ps3_control", 1, subPS3);
  ros::Subscriber subM = node.subscribe("control_mode", 1, subMode);

  // Loop rate //
  ros::Rate loop_rate(10);

  // Max percentage thrust //

  Eigen::Vector3d vecZ;
  Eigen::Vector3d dVel, dAvel;
  vecZ(0)=0;
  vecZ(1)=0;
  vecZ(2)=0.0141;


  // Initializing vectors and defining parameters//

  all_vects_zero();
  wrench_zero(F);
  wrench_zero(FOld1);
  wrench_zero(FOld2);

  intZ = 0;

  geometry_msgs::Wrench Fz;

  //////////////////////////

  while (ros::ok())
  {
    if(resetInt)
    {
        intZ = 0;
        resetInt = true;
    }
    ros::spinOnce();
    if(mode == 1)
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

      if(fabs(force(2))<2.4 && fabs(force(1))<1.2 && fabs(force(0))<1.2) // increasing only if the command is not saturating //
     {
        intZ+= (pos(2)-posdesir(2))/10;
        if(fabs(intZ*vecZ(2))>0.6){intZ=copysign(0.6/vecZ(2),intZ);}

      }


              // PID //

      forceGlobF(0)=-(1.1*dPos(0) + 0.2*(limitVel((dPos(0)-dPosOld1(0))*10.0,1.0)));
      forceGlobF(1)=posdesir(1);
      forceGlobF(2)=-(1.5*dPos(2) + 1.0*(limitVel((dPos(2)-dPosOld1(2))*10.0,1.0)))+intZ*0.038;

      torque(0)=0;
      torque(1)=0;
      torque(2)=1.0*dAngle(2) + 2.0*(limitVel((dAngle(2)-dAngleOld1(2))*10.0,1.0));



      ////////////////////////////////////


      // Rotate Force into body-frame //
      //force=Rmatrix.inverse()*forceGlobF;
      //intFz=Rmatrix.inverse()*intFzGlbf;

      force=forceGlobF;

      // Command //
      F.force.x=force(0);
      F.force.y=force(1);
      F.force.z=force(2);
      F.torque.x=torque(0);
      F.torque.y=torque(1);
      F.torque.z=torque(2);

      // Update //
      torqueOld1=torque;

      forceOld1=force;

      dPosOld1=dPos;

      dAngleOld1=dAngle;

      FOld1=F;


      /////////////////////////////////

      Controle_node.publish(F);

      ROS_INFO("pos x: %f, y: %f ,z: %f ",pos(0),pos(1),pos(2));
      ROS_INFO("angle x: %f, y: %f ,z: %f ",angle(0),angle(1),angle(2));
      ROS_INFO("Dangle x: %f, y: %f ,z: %f ",dAngle(0),dAngle(1),dAngle(2));
      ROS_INFO("fx: %f, fy: %f, fz: %f,Tx: %f, Ty: %f, Tz: %f",F.force.x, F.force.y, F.force.z, F.torque.x, F.torque.y, F.torque.z);

      loop_rate.sleep();
    }
    if (mode == 0) //illogical I know
    {
       Controle_node.publish(ps3Wrench);
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

