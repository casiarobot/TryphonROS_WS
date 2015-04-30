// Standard C/C++ libraries
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <ctime>
#include <iostream>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <net/if.h>
#include <arpa/inet.h>

// YAML file
#include <YAMLParser.h>



//library for ros
#include <ros/ros.h>
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Pose.h"

//library for Android msg
#include <sensor_msgs/Imu.h>

//libraries for the sonar
#include "sensors/sonar.h"
#include "sensors/sonarArray.h"
#include "sensors/compass.h"
#include "sensors/imuros.h"
#include "state/state.h"

//libraries for the array
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"

//libraries for the control
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include <sstream>

#include <fcntl.h>
#include <time.h>
#include <unistd.h>
#include <errno.h>

//#include "i2c-dev.h"
//#include "motors.h"
#include "sensors/motor.h"
#include "sensors/motorArray.h"
//#include "cube.h"       // Cube geometry, inertia, etc.


bool start=true;

double dsx1=0;
double dsy1=0;
double rz=0;
double rz0=0;
double ax=0;
double ay=0;
double az=0;
double gx=0;
double gy=0;
double gz=0;
double mx=0;
double my=0;
double mz=0;


double hmax=6000;
double x[3]={0,0,0};
double y[3]={0,0,0};
double z[3]={0,0,0};
double xf[3]={0,0,0};
double yf[3]={0,0,0};
double zf[3]={0,0,0};

double q0[3]={0,0,0};
double q1[3]={0,0,0};
double q2[3]={0,0,0};
double q3[3]={0,0,0};
double q0f[3]={0,0,0};
double q1f[3]={0,0,0};
double q2f[3]={0,0,0};
double q3f[3]={0,0,0};

double phi[3]={0,0,0};
double theta[3]={0,0,0};
double psi[3]={0,0,0};
double phif[3]={0,0,0};
double thetaf[3]={0,0,0};
double psif[3]={0,0,0};


void subImu(const sensors::imuros::ConstPtr& imudata)
{
  ax=imudata->accel[0];
  ay=imudata->accel[1];
  az=imudata->accel[2];
  gx=imudata->gyro[0];
  gy=imudata->gyro[1];
  gz=imudata->gyro[2];
  mx=imudata->magn[0];
  my=imudata->magn[1];
  mz=imudata->magn[2];

  //ROS_INFO("ax: %f, ay: %f, az: %f",ax,ay,az);
  //ROS_INFO("gx: %f, gy: %f, gz: %f",gx,gy,gz);
  //ROS_INFO("mx: %f, my: %f, mz: %f",mx,my,mz);
}

void update( double &v[3])
{
  for(int i=0;i<2;i++){v[i]=v[i+1];}
}


void subMCPTAM(const geometry_msgs::PoseArray Aposes) // with MCPTAM
{

  update(x);
  update(y);
  update(z);
  update(xf);
  update(yf);
  update(zf);
  update(q1);
  update(q2);
  update(q3);
  update(q0);
  update(q1f);
  update(q2f);
  update(q3f);
  update(q0f);
  update(phi);
  update(theta);
  update(psi);
  update(phif);
  update(thetaf);
  update(psif);



  geometry_msgs::Pose pose=Aposes.poses[0];
  x[2]=pose.position.x;
  y[2]=pose.position.y;
  z[2]=pose.position.z;

  q1[2]=pose.orientation.x;
  q2[2]=pose.orientation.y;
  q3[2]=pose.orientation.z;
  q0[2]=pose.orientation.w;

  phi[2]=atan2(2*(q0[2]*q1[2]+q2[2]*q3[2]),1-2*(q1[2]*q1[2]+q2[2]*q2[2]));
  theta[2]=asin(2*(q0[2]*q2[2]-q3[2]*q1[2]));
  psi[2]=atan2(2*(q0[2]*q3[2]+q1[2]*q2[2]),1-2*(q3[2]*q3[2]+q2[2]*q2[2]));

  if(start){start=false;}

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
  char rosname[100];
  std::string temp_arg;
  //gethostname(rosname,100);
  sprintf(rosname,"state_estimator_%s",get_ip());
  ros::init(argc, argv, rosname);
  ros::NodeHandle node;

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

  sprintf(rosname,"/%s/state",temp_arg.c_str());
  ros::Publisher Controle_node = node.advertise<state::state>(rosname,1);
  ros::Rate loop_rate(10);
  geometry_msgs::Pose pose;
  state::state state;
  int print=0;

  //ros::Subscriber subA = node.subscribe("/android/imu", 1, poseCallback);
  //sprintf(rosname,"/%s/sonars",temp_arg.c_str());
  //ros::Subscriber subS = node.subscribe(rosname, 1, subSonar);
  sprintf(rosname,"/%s/compass",temp_arg.c_str());
  ros::Subscriber subC = node.subscribe(rosname,1,subComp);
  /*sprintf(rosname,"/%s/imbuf",temp_arg.c_str());
    ros::Subscriber subI = node.subscribe(rosname,1,subImu);*/
  //ros::Subscriber subSick = node.subscribe("/cubeA_pose", 1, poseCallback);
  ros::Subscriber subM = node.subscribe("mcptam/tracker_pose_array",1,subMCPTAM);

  Eigen::Vector3d xState,yState,zState;
  Egen::Vector quatState; // quaternion and angular velocity





  while (ros::ok())
  {
    ////////////////////////////////////
    ////       State estimator      ////
    ////////////////////////////////////

    // Predict //
    if(imu || mcp)
    {

    }
    if(imu && mcp)
    {

    }
    if(!imu && mcp)
    {

    }
    if(imu && !mcp)
    {

    }
    if(!imu && !mcp)
    {

    }





    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
