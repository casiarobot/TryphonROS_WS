// Standard C/C++ libraries
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <ctime>
#include <unistd.h>
#include <string.h> /* for strncpy */

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <net/if.h>
#include <arpa/inet.h>
#include <fstream>

//library for ros
#include <ros/ros.h>
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"

//library for Android msg
#include <sensor_msgs/Imu.h>

//libraries for the sonar
#include "sensors/sonar.h"
#include "sensors/sonarArray.h"
#include "sensors/compass.h"
#include "sensors/imuros.h"
#include "sensors/imubuff.h"
#include "state/state.h"

//libraries for the array
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"

//libraries for the control
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
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


#include <Eigen/Dense>
#include <Eigen/Geometry>

bool start=true;
bool startA=true;
bool startB=true;
double dsz1=0;
double dsz2=0;
double dsz3=0;
double dsz4=0;
double dsxt[5]={0,0,0,0,0};
double dsyt[5]={0,0,0,0,0};
double dszt[5]={0,0,0,0,0};
double dsrt[5]={0,0,0,0,0};
double dsxtf[5]={0,0,0,0,0};
double dsytf[5]={0,0,0,0,0};
double dsztf[5]={0,0,0,0,0};
double dsrtf[5]={0,0,0,0,0};
//double dszt=0;
//double dszto=0;
double dsx1=0,dsx2=0,dsx3=0;
double dsy1=0,dsy2=0,dsy3=0;
double rz1=0,rz2=0,last_rz2=0, rz2_init=0;
double rz0=0;
double rzold=0;
double ax=0;
double ay=0;
double az=0;
double gx=0;
double gy=0;
double gz=0;
double mx=0;
double my=0;
double mz=0;
double debutA,debutB;

using namespace std;

std::ofstream myfile;
std::ofstream myfile1;

double hmax=6000;


template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}


void subSonar(const sensors::sonarArray::ConstPtr& msg)
{
    dsz1=0;
    dsz2=0;
    dsz3=0;
    dsz4=0;
    dsx1=0;
    dsy1=0;
    int okdsz=0;
    /*
	for(int i=0; i<4;i++){
      	dsxt[i]=dsxt[i+1];
		dsxtf[i]=dsxtf[i+1];
		dsyt[i]=dsyt[i+1];
		dsytf[i]=dsytf[i+1];
		dszt[i]=dszt[i+1];
		dsztf[i]=dsztf[i+1];
	}
	*/
    //dszto=dszt;
    for (int i=0; i<msg->sonars.size(); ++i)
    {
        const sensors::sonar &sonar = msg->sonars[i];
        //ROS_INFO_STREAM("ID: " << sonar.id << " - D0: " << sonar.distance[0] <<
        //	               ", D1: " << sonar.distance[1]);

        // Average but not dividing by ten to convert cm into mm
        if (sonar.id == (int)(0xE0)/2){
            for (int j=0;j<10;j++)
            	dsx1 -= sonar.distance / 10;
        }
        else if (sonar.id == (int)(0xE6)/2){
            for (int j=0;j<10;j++)
            	dsy1 += sonar.distance / 10;
        }
        else if (sonar.id == (int)(0xF8)/2){
            for (int j=0;j<10;j++)
            	dsz3 += sonar.distance / 10;
			if(dsz3>hmax)
				dsz3 = 0;
			else 
				++okdsz;
		}
        else if (sonar.id == (int)(0xFA)/2){
            for (int j=0;j<10;j++)
            	dsz4 += sonar.distance / 10;
            if(dsz4>hmax)
				dsz4 = 0;
            else 
				++okdsz;
		}
        else if (sonar.id == (int)(0xFC)/2){
            for (int j=0;j<10;j++)
            	dsz1 += sonar.distance / 10;
            if(dsz1>hmax)
				dsz1 = 0;
            else 
				++okdsz;
		}
        else if (sonar.id == (int)(0xFE)/2){
            for (int j=0;j<10;j++)
            	dsz2 += sonar.distance / 10;
            if(dsz2>hmax)
				dsz2 = 0;
            else 
				++okdsz;
		}

    }
    dsxt[4]=dsx1;dsyt[4]=dsy1;
    if(okdsz!=0){dszt[4]=(dsz1+dsz2+dsz3+dsz4)/okdsz;}
    else {dszt[4]=dszt[3];}
    //ROS_INFO("distance 1: %f, distance 2: %f, distance x : %f, distance y : %f",dsz1,dsz2,dsx1,dsy//
}

void subComp(const sensors::compass::ConstPtr& msg)
{
    //ROS_INFO_STREAM("ID: " << msg->id << " - RZ0: " << msg->rz[0] <<
    //                ", RZ1: " << msg->rz[1]);
        for(int i=0; i<4;i++){
      	dsrt[i]=dsrt[i+1];
	dsrtf[i]=dsrtf[i+1];}

    if (msg->id == (int)(0xC0)/2){
	rz1=(msg->rz[0])-rz0;}
    if (start && rz1!=0){rz0=rz1;
	start=false;}
    dsrt[4]=rz1;
    //ROS_INFO("rotation: %f",rz);
}

void subImu(const sensors::imubuff::ConstPtr& msg)
{
ax=msg->buffer[0].accel[0];
ay=msg->buffer[0].accel[1];
az=msg->buffer[0].accel[2];
gx=msg->buffer[0].gyro[0];
gy=msg->buffer[0].gyro[1];
gz=msg->buffer[0].gyro[2];
mx=msg->buffer[0].magn[0];
my=msg->buffer[0].magn[1];
mz=msg->buffer[0].magn[2];

//ROS_INFO("ax: %f, ay: %f, az: %f",ax,ay,az);
//ROS_INFO("gx: %f, gy: %f, gz: %f",gx,gy,gz);
//ROS_INFO("mx: %f, my: %f, mz: %f",mx,my,mz);
}

void poseCallback(geometry_msgs::PoseStamped ps){
  if(startA)
  {
    debutA=ros::Time::now().toSec();
    startA=false;
  }

  if(ps.pose.position.x==ps.pose.position.x)
    dsx2=ps.pose.position.x;
  if(ps.pose.position.y==ps.pose.position.y)
    dsy2=ps.pose.position.y;

  //rz2=-sgn(ps.pose.orientation.z)*ps.pose.orientation.w;
  Eigen::Quaterniond quad(ps.pose.orientation.x,ps.pose.orientation.y,ps.pose.orientation.z,ps.pose.orientation.w);
  float rz2_tmp = atan2(quad.toRotationMatrix()(1, 2), quad.toRotationMatrix()(2, 2));
  if(rz2_tmp == rz2_tmp) //Testing if NaN
    rz2=rz2_tmp;


  double secs = ros::Time::now().toSec();
  myfile << dsx2 << ","<< dsy2 <<","<<secs-debutA<<endl;
}

void poseCallback2(geometry_msgs::PoseStamped ps){
  if(startB)
  {
    debutB=ros::Time::now().toSec();
    startB=false;
  }

  if(ps.pose.position.x==ps.pose.position.x)
    dsx3=ps.pose.position.x;
  if(ps.pose.position.y==ps.pose.position.y)
    dsy3=ps.pose.position.y;

  //rz2=-sgn(ps.pose.orientation.z)*ps.pose.orientation.w;
  Eigen::Quaterniond quad(ps.pose.orientation.x,ps.pose.orientation.y,ps.pose.orientation.z,ps.pose.orientation.w);
  float rz2_tmp = atan2(quad.toRotationMatrix()(1, 2), quad.toRotationMatrix()(2, 2));
  if(rz2_tmp == rz2_tmp) //Testing if NaN
    rz2=rz2_tmp;

  double secs = ros::Time::now().toSec();
  myfile << dsx3 << ","<< dsy3 <<","<<secs-debutB<<endl;
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
  //char rosname[100];
	//std::string temp_arg;
	//gethostname(rosname,100);
	//sprintf(rosname,"state_estimator_%s",get_ip());
	ros::init(argc, argv,"state_sub"); //rosname);
  ros::NodeHandle node;
	int pos_src=0;
	/*
	if (argc==3)
        {
	  pos_src=atoi(argv[2]);
          ROS_INFO("TARGET IS: %s AND POSE SOURCE: %i", argv[1], pos_src);
        }
        else
        {
          ROS_ERROR("Failed to get param 'target' and 'pose source'");
	  	return 0;
        }
	temp_arg = argv[1];
	std::replace(temp_arg.begin(), temp_arg.end(), '.', '_');

	sprintf(rosname,"/%s/state",temp_arg.c_str());
    	ros::Publisher Controle_node = node.advertise<state::state>(rosname,1);
    	ros::Rate loop_rate(10);
    	geometry_msgs::Pose pose;
    	state::state state;
	int print=0;*/
  char link[100]="//home/py/Dropbox/Tryphon_PPY/Residency_nov2014/Matlab/csvfiles/Thrust85/SICK/";// make sure it is the right path
  char link1[100]="/home/py/Dropbox/Tryphon_PPY/Residency_nov2014/Matlab/csvfiles/Thrust85/SICK/";

  strcat(link,argv[1]);
  strcat(link1,argv[1]);

  strcat(link,"_poseA.csv");
  strcat(link1,"_poseB.csv");

  myfile.open(link);// make sure it is the right path
  myfile1.open(link1);// make sure it is the right path

  myfile << "Run"<< endl << "dx,dy,time" << endl  ;
  myfile1 << "Run"<< endl << "dx,dy,time" << endl  ;

	/*
	//ros::Subscriber subA = node.subscribe("/android/imu", 1, poseCallback);
	sprintf(rosname,"/%s/sonars",temp_arg.c_str());
	ros::Subscriber subS = node.subscribe(rosname, 1, subSonar);
	sprintf(rosname,"/%s/compass",temp_arg.c_str());
	ros::Subscriber subC = node.subscribe(rosname,1,subComp);
	sprintf(rosname,"/%s/imbuf",temp_arg.c_str());
	ros::Subscriber subI = node.subscribe(rosname,1,subImu);*/
	ros::Subscriber subSick = node.subscribe("/cubeA_pose", 1, poseCallback);
  ros::Subscriber subSick2 = node.subscribe("/cubeB_pose", 1, poseCallback2);


	while (ros::ok())
	{
    ros::spinOnce();
  }
  myfile.close();
  myfile1.close();
	return 0;
}
