#include "controls.h"
#include "ctr_fuzzy.h"
/*#define DEVICE_I2C      "/dev/i2c-3"
typedef enum {FALSE, TRUE}
BOOLEAN;
typedef unsigned char BYTE;
#define INTERACTION     0
*/


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
geometry_msgs::Wrench F;
double thetaorig=0;
double theta=0;
double thetadesir=0;
Eigen::Vector4d quatdesir, quatorig, quatglobf;
Eigen::Vector3d forceglobf, forcecubef, forcecubefold, torqglobf, torqcubef, torqcubefold, posdesir, posorig, posglobf, veldesir, velorig, velglobf, aveldesir, avelorig, avelglobf;
bool start=true;
double Gprop,Gderiv;


void subState(const state::state state)
{
  //Starting Pose//
  if(start){
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
    start=false;
    ROS_INFO("Init pose done");
  }
  posglobf(0)=state.pos[0]-posorig(0);
  posglobf(1)=state.pos[1]-posorig(1);
  posglobf(2)=state.pos[2]-posorig(2);
  quatglobf(0)=state.quat[0];
  quatglobf(1)=state.quat[1];
  quatglobf(2)=state.quat[2];
  quatglobf(3)=state.quat[3];
  velglobf(0)=state.vel[0];
  velglobf(1)=state.vel[1];
  velglobf(2)=state.vel[2];
  avelglobf(0)=state.angvel[0];
  avelglobf(1)=state.angvel[1];
  avelglobf(2)=state.angvel[2];
  thetaorig=atan2(2*(state.quat[0]*state.quat[3]+state.quat[1]*state.quat[2]),1-2*(state.quat[2]*state.quat[2]+state.quat[3]*state.quat[3]));

  // State estimation //
  /*dsx=state.pos[0];
    dsy=state.pos[1];
    dsz=state.pos[2];
    state.vel[0];
    state.vel[1];
    rz=state.quat[2];
    //pose.orientation.w;

	// Zero compass//
	if(start){
    	rz0=rz;
    	start=false;}

    	// Z-Axis //
	erroro=errorn;
    errorn=dszwant-dsz;
    deriv=(errorn-erroro)/0.05;
    */
}


void subdPose(const geometry_msgs::Pose dPose)
{

  if(start)
    return;

  if(dPose.position.x+posorig(0)<=MAX_X && dPose.position.x+posorig(0)>=MIN_X)
    posdesir(0)=posorig(0)+dPose.position.x;

  if(dPose.position.y+posorig(1)<=MAX_Y && dPose.position.y+posorig(1)>=MIN_Y)
    posdesir(1)=posorig(1)+dPose.position.y;

  if(dPose.position.z+posorig(2)<=MAX_Z && dPose.position.z+posorig(2)>=MIN_Z)
    posdesir(2)=posorig(2)+dPose.position.z;

  /*Eigen::Quaterniond quad(dPose.orientation.x,dPose.orientation.y,dPose.orientation.z,dPose.orientation.w);
  float ang = atan2(quad.toRotationMatrix()(1, 2), quad.toRotationMatrix()(2, 2));*/

  quatdesir(0)=quatorig(0)+dPose.orientation.x;


  //ROS_INFO("Dx: %f, Dy: %f, Dz: %f,DTx: %f, DTy: %f, DTz: %f",des_state.pos[0], des_state.pos[1], des_state.pos[2], des_state.quat[1], des_state.quat[2], des_state.quat[0]);
}

void callback(controls::controlConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: x: %f, y: %f, z: %f, yaw: %f  ",
           config.x,
           config.y,
           config.z,
           config.yaw);
  posdesir(0)=config.x,
  posdesir(1)=config.y;
  posdesir(2)=config.z;
  thetadesir=config.yaw;
  Gprop=config.prop;
  Gderiv=config.deriv;
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
  vect3_zero(forceglobf);
  vect3_zero(forcecubef);
  vect3_zero(torqglobf);
  vect3_zero(torqcubef);
  vect3_zero(posdesir);
  vect3_zero(posorig);
  vect3_zero(posglobf);
  vect4_zero(quatdesir);
  vect4_zero(quatorig);
  vect4_zero(quatglobf);
  vect3_zero(veldesir);
  vect3_zero(velorig);
  vect3_zero(velglobf);
  vect3_zero(aveldesir);
  vect3_zero(avelorig);
  vect3_zero(avelglobf);

}

inline float IIR(float old, float in, float gain)
{
return old + (1.0-gain)*(in-old);
}


ros::Publisher Controle_node;

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
  sprintf(rosname,"/%s/command_control",temp_arg.c_str());
  Controle_node = node.advertise<geometry_msgs::Wrench>(rosname,1);
  ros::Rate loop_rate(10); //CHANGE TIME OF FUZZY CONTROL OF DIFF zOF 5H

  sprintf(rosname,"/%s/state",temp_arg.c_str());
  ros::Subscriber subS = node.subscribe(rosname, 1, subState);
  dynamic_reconfigure::Server<controls::controlConfig> server;
  dynamic_reconfigure::Server<controls::controlConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  //ros::Subscriber subdP = node.subscribe("/desired_deltapose", 1, subdPose);
  Eigen::Matrix3d Rmatrix, PropMatrix, DerivMatrix;
  PropMatrix << 1, 0, 0,
  0, 1, 0,
  0, 0, 1;

  DerivMatrix<< 1, 0, 0,
  0, 1, 0,
  0, 0, 1;
  //ros::Subscriber subA = node.subscribe("/android/imu", 1, poseCallback);

  all_vects_zero();

  Eigen::Quaterniond q;
  double kp[3],kd[3];
  double gain = exp(-10/2); //gain = exp(-SamplingPeriod/FilterTimeConstant);
  posdesir(2)=1;
  while (ros::ok())
  {
    ros::spinOnce();
    ////////////////////////////////////
    ////       Controller           ////
    ////////////////////////////////////
    q.x()=quatglobf(0);
    q.y()=quatglobf(1);
    q.z()=quatglobf(2);
    q.w()=quatglobf(3);

    /*if(fabs(posdesir(0)-posglobf(0))<0.4)
    {
      kp[0]=0.3;
      kd[0]=0.8;
    }
    else
    {
      kp[0]=0.2;
      kd[0]=0.6;
    }

    if(fabs(posdesir(1)-posglobf(1))<0.4)
    {
      kp[0]=0.3;
      kd[0]=0.8;
    }
    else
    {
      kp[0]=0.2;
      kd[0]=0.6;
    }

    if(fabs(posdesir(2)-posglobf(2))<0.4)
    {
      kp[2]=0.5;
      kd[2]=1;
    }
    else
    {
      kp[2]=0.3;
      kd[2]=0.8;
    }


    PropMatrix << kp[0], 0, 0,
    0, kp[1], 0,
    0, 0, kp[2];

    DerivMatrix<< kd[0], 0, 0,
    0, kd[1], 0,
    0, 0, kd[2];*/

    Rmatrix<<cos(-M_PI*0.5), -sin(-M_PI*0.5), 0,
             sin(-M_PI*0.5), cos(-M_PI*0.5), 0,
             0, 0, 1;//q.toRotationMatrix();

    std::cout<<"Rotation matrix :"<<Rmatrix<<std::endl;

    forceglobf=Gprop*PropMatrix*(posdesir-posglobf)-Gderiv*DerivMatrix*(velglobf);
    //torqglobf(2)=-0.012*(quatdesir(0)-quatglobf(0));
    forcecubefold=forcecubef;
    forcecubef=Rmatrix*forceglobf;


    F.force.x=IIR(forcecubefold(0),forcecubef(0),gain);
    F.force.y=IIR(forcecubefold(1),forcecubef(1),gain);
    F.force.z=IIR(forcecubefold(2),forcecubef(2),gain);//0.0012*(errorn+1.2*deriv);
    F.torque.x=0;
    F.torque.y=0;
    F.torque.z=-1.2*sin(theta-thetaorig);


    /////////////////////////////////
    ROS_INFO("fx: %f, fy: %f, fz: %f,Tx: %f, Ty: %f, Tz: %f ",F.force.x,F.force.y,F.force.z,F.torque.x,F.torque.y,F.torque.z);
    ROS_INFO("angle : %f, angle error : %f ",theta,theta-thetaorig);

    Controle_node.publish(F);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}




