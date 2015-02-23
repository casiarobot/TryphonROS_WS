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
Eigen::Vector3d vel, avel;
Eigen::Vector3d CIMUpos(1,0,-1.125); // vector postion from Center of mass to IMU in tryphon frame
Eigen::Matrix3d Rmatrix, CPmatrix, RIMUmatrix;
Eigen::Quaterniond quatIMU(0.99255, 0,0.12187, 0); // quat of the rotation matrix between the tryphon frame and the IMU frame



bool start,path;
double Gprop,Gderiv;
double gain = exp(-0.1/1); //gain = exp(-SamplingPeriod/FilterTimeConstant);
double fbuoyCoeff=1;       // gazebo parameter
int pathNb=0;              // Path nb wanted




void subState(const state::state state)
{
  /*//Starting Pose needed only when the inertial frame is not centered//
  if(start==true){
    posorig(0)=state.pos[0];
    posorig(1)=state.pos[1];
    posorig(2)=state.pos[2];
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
    CPmatrix<< 0, -CIMUpos(2), CIMUpos(1),
    CIMUpos(2), 0, -CIMUpos(0),
    -CIMUpos(1), CIMUpos(0), 0;
    start=false;
  }

  pos(0)=Pose.position.x; // defined in global frame
  pos(1)=Pose.position.y;
  pos(2)=Pose.position.z;
  Eigen::Quaterniond quat(Pose.orientation.w,Pose.orientation.x,Pose.orientation.y,Pose.orientation.z);
  quat=quatIMU.inverse()*quat; // compute the quaternion between the vision world and the tryphon frame


  angle(0)=atan2(2*(quat.w()*quat.x()+quat.y()*quat.z()),1-2*(quat.x()*quat.x()+quat.y()*quat.y()));
  angle(1)=asin(2*(quat.w()*quat.y()-quat.z()*quat.x()));
  angle(2)=atan2(2*(quat.w()*quat.z()+quat.x()*quat.y()),1-2*(quat.z()*quat.z()+quat.y()*quat.y()));
  Rmatrix=quat.toRotationMatrix();

  pos=pos-Rmatrix*CIMUpos;  // offset due to the fact that the pose is the one of the IMU


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

    vel=vel+Rmatrix*CPmatrix*avel; // compute the vel of the center of mass

  }
}


/*void subdP(const geometry_msgs::Pose dPose)
{

  if(start)posdesir
    return;

  if(dPose.position.x+posorig(0)<=MAX_X && dPose.position.x+posorig(0)>=MIN_X)
    posdesir(0)=posorig(0)+dPose.position.x;

  if(dPose.position.y+posorig(1)posdesir<=MAX_Y && dPose.position.y+posorig(1)>=MIN_Y)
    posdesir(1)=posorig(1)+dPose.position.y;

  if(dPose.position.z+posorig(2)<=MAX_Z && dPose.position.z+posorig(2)>=MIN_Z)
    posdesir(2)=thetaold[0]=thetaold[1];
  thetaold[1]=thetaold[2];
  thetaold[2]=theta;r(2)=posorig(2)+dPose.position.z;

  /*Eigen::Quaterniond quad(dPose.vecZ(0)=0;
  orientation.x,dPose.orientation.y,dPose.orientation.z,dPose.orientation.w);
  float ang = atan2(quad.toRotationMatrix()(1, 2), quad.toRotationMatrix()(2, 2));*/
/*
  quatdesir(0)=quatorig(0)+dPose.orientation.x;


  //ROS_INFO("Dx: %f, Dy: %f, Dz: %f,DTx: %f, DTy: %f, DTz: %f",des_state.pos[0], des_state.pos[1], des_state.pos[2], des_state.quat[1], des_state.quat[2], des_state.quat[0]);
}*/

void callback(controls::controlConfig &config, uint32_t level) {
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
  Gprop=config.prop;
  Gderiv=config.deriv;

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
{vect3_zero(posdesir);
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
  vect3_zero(posdesir);
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
{
  return 1.16826*u1-0.42411820*u2+0.0639643*e+0.127929*e1+0.0639643*e2; // fc=1hz
  //return 1.56450*u1-0.6436623*u2+0.0197896*e+0.03957916533*e1+0.0197896*e2; // fc=0.5hz

}


ros::Publisher Controle_node;
ros::Publisher Desired_pose_node;
ros::Publisher Pose_node;
ros::Publisher Vel_node;
ros::Publisher Fbuoy_node;
ros::Publisher Controle_notfiltered_node;


void mySigintHandler(int sig)
{
  // Do some custom action.
  // For example, publish a stop message to some other nodes.
  F.force.x=0;F.force.y=0;F.force.z=0;F.torque.x=0;F.torque.y=0;F.torque.z=0;
  Controle_node.publish(F);
  ROS_INFO("fx: %f, fy: %f, fz: %f,Tx: %f, Ty: %f, Tz: %f",F.force.x, F.force.y, F.force.z, F.torque.x, F.torque.y, F.torque.z);
  // All the default sigint handler does is call shutdown()
  ros::shutdown();
}

double modulo(double f)
{
  if(f>M_PI)
  {
   f=f-2*M_PI;
  }
  else
  {
    if(f<-M_PI)
    {
      f=f+2*M_PI;
    }
  }
  return f;
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
  sprintf(rosname,"/%s/command_control_filtered",temp_arg.c_str());
  Controle_notfiltered_node = node.advertise<geometry_msgs::Wrench>(rosname,1);
  sprintf(rosname,"/%s/desired_pose",temp_arg.c_str());
  Desired_pose_node = node.advertise<geometry_msgs::Pose>(rosname,1);
  sprintf(rosname,"/%s/pose",temp_arg.c_str());
  Pose_node = node.advertise<geometry_msgs::Pose>(rosname,1);
  sprintf(rosname,"/%s/velocity",temp_arg.c_str());
  Vel_node = node.advertise<geometry_msgs::Pose>(rosname,1);



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




  // LQR Gain Matrices //
  Eigen::Matrix3d PropMatrixforce, DerivMatrixforce, PropMatrixtorque, DerivMatrixtorque;
  Eigen::Vector3d vecZ;

  PropMatrixforce << 2.6458, 0, 0,
  0, 2.6458, 0,
  0, 0, 2.7259;

  DerivMatrixforce << 11.3448, 0, 0,
  0, 11.3448, 0,
  0, 0, 11.506;

  PropMatrixtorque << 0.1716, 0, 0,
  0, 0.1716, 0,
  0, 0, 2.6458;

  DerivMatrixtorque << 3.5052, 0, 0,
  0, 3.5150, 0,
  0, 0, 9.5741;



  vecZ(0)=0;
  vecZ(1)=0;
  vecZ(2)=0.0187;

  // Defining paths //

  std::vector<Eigen::Vector4d> pathLine;
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
  pathSquare.push_back(pS);

  std::vector<Eigen::Vector4d> pathSquare2;


  std::vector< std::vector<Eigen::Vector4d> > paths;

  paths.push_back(pathLine);
  paths.push_back(pathLine2);
  paths.push_back(pathSquare);
  //paths.push_back(pathSquare2);


  //////////////////////

  //ros::Subscriber subA = node.subscribe("/android/imu", 1, poseCallback);
  // Initializing vectors and defining parameters//

  all_vects_zero();
  wrench_zero(F);
  wrench_zero(FOld1);
  wrench_zero(FOld2);


  start=true;
  path=false;

  int step=0;
  double intZ = 0;
  geometry_msgs::Wrench FUnfiltered;
  Eigen::Vector4d range(0.10,0.10,0.10,0.1);

  double maxFxy=1.24;
  double minFxy=-0.64;
  double maxFz=2.48;
  double minFz=-1.28;
  double maxT=0.64;
  double minT=-0.64;


  /////////////////////////



  while (ros::ok())
  {
    ros::spinOnce();

    if(!start)
    {
      // Path //
      if(path)
      {
        if(fabs(pos(0)-posdesir(0))<range(0) && fabs(pos(1)-posdesir(1))<range(1) && fabs(pos(2)-posdesir(2))<range(2) && fabs(angle(2)-angledesir(2))<range(3))
        {
          if(step<paths[pathNb].size()-1){step++;}
          else{step=0;}

        }
        posdesir(0)=paths[pathNb][step](0);
        posdesir(1)=paths[pathNb][step](1);
        posdesir(2)=paths[pathNb][step](2);
        angledesir(2)=paths[pathNb][step](3);
      }
      else{step=0;}

    // Integral term //
      if(fabs(force(2))<1.2) // increasing only if the command is not saturating //
     {
        intZ+= pos(2)-posdesir(2);
        if(fabs(intZ*vecZ(2))>0.64){intZ=copysign(3.2,intZ);}

      }



      // avoiding the discontunity due to the angle around pi and -pi //
      dAngle(0)=modulo(angle(0)-angledesir(0));
      dAngle(1)=modulo(angle(1)-angledesir(1));
      dAngle(2)=modulo(angle(2)-angledesir(2));
      dPos=pos-posdesir;

      ////////////////////////////////////
      ////       Controller           ////
      ////////////////////////////////////

      // LQR //
      forceGlobF=-( PropMatrixforce*(dPos) + DerivMatrixforce*(vel) + intZ*vecZ);
      torque=-(PropMatrixtorque*(dAngle)+DerivMatrixtorque*(avel));

      // PID //

//       forceGlobF(0)=0.2397*forceGfOld1(0)                        +262.6*-dPos(0)  -260.6*-dPosOld1(0);
//       forceGlobF(1)=0.2397*forceGfOld1(1)                        +262.6*-dPos(1)  -260.6*-dPosOld1(1);
//       forceGlobF(2)=1.2400*forceGfOld1(2) -0.2397*forceGfOld2(2) +262.6*-dPos(2)  -523.2*-dPosOld1(2)  +260.6*-dPosOld2(2);

//       torque(0)=0.2397*torqueOld1(0)  +20.2*-dAngle(0) -20.05*-dAngleOld1(0);
//       torque(1)=0.2397*torqueOld1(1)  +20.2*-dAngle(1) -20.05*-dAngleOld1(1);
//       torque(2)=0.1353*torqueOld1(2)  +202*-dAngle(2)  -200.3*-dAngleOld1(2);


      ////////////////////////////////////

      // Saturation //
      forceGlobF(0)=saturation(forceGlobF(0),maxFxy,-maxFxy);
      forceGlobF(1)=saturation(forceGlobF(1),maxFxy,-maxFxy);
      forceGlobF(2)=saturation(forceGlobF(2),maxFz,-maxFz);


      // Rotate Force into body-frame //
      force=Rmatrix.inverse()*forceGlobF;


      // Saturation //
      force(0)=saturation(force(0),maxFxy,minFxy);
      force(1)=saturation(force(1),maxFxy,minFxy);
      force(2)=saturation(force(2),maxFz,minFz);
      torque(0)=saturation(torque(0),maxT,minT);
      torque(1)=saturation(torque(1),maxT,minT);
      torque(2)=saturation(torque(2),maxT,minT);


      //The first saturation is there to avoid the issue arising when the saturation function is applied the second time.


      // Command //
      F.force.x=smooth(FOld1.force.x,FOld2.force.x,force(0),forceOld1(0),forceOld2(0));
      F.force.y=smooth(FOld1.force.y,FOld2.force.y,force(1),forceOld1(1),forceOld2(1));
      F.force.z=smooth(FOld1.force.z,FOld2.force.z,force(2),forceOld1(2),forceOld2(2));
      F.torque.x=smooth(FOld1.torque.x,FOld2.torque.x,torque(0),torqueOld1(0),torqueOld2(0));
      F.torque.y=smooth(FOld1.torque.y,FOld2.torque.y,torque(1),torqueOld1(1),torqueOld2(1));
      F.torque.z=smooth(FOld1.torque.z,FOld2.torque.z,torque(2),torqueOld1(2),torqueOld2(2));


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
      Pose_node.publish(fPose);
      Desired_pose_node.publish(desirPose);
      Vel_node.publish(fVel);

      FUnfiltered.force.x=force(0);
      FUnfiltered.force.y=force(1);
      FUnfiltered.force.z=force(2);
      FUnfiltered.torque.x=torque(0);
      FUnfiltered.torque.y=torque(1);
      FUnfiltered.torque.z=torque(2);
      Controle_notfiltered_node.publish(FUnfiltered);
    }

    geometry_msgs::Wrench info;
    info.force.x=fbuoyCoeff;
    Fbuoy_node.publish(info);

    loop_rate.sleep();
  }
  return 0;
}




