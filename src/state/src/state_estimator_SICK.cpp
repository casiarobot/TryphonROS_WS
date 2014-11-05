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
double dsz1=0;
double dsz2=0;
double dsz3=0;
double dsz4=0;
double dszt[5]={0,0,0,0,0};
double dsztf[5]={0,0,0,0,0};
//double dszt=0;
//double dszto=0;
double dsx1=0,dsx2=0;
double dsy1=0,dsy2=0;
double rz1=0,rz2=0;
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
    for(int i=0; i<4;i++){dszt[i]=dszt[i+1];
	dsztf[i]=dsztf[i+1];}
    //dszto=dszt;
    for (int i=0; i<msg->sonars.size(); ++i)
    {
        const sensors::sonar &sonar = msg->sonars[i];
        //ROS_INFO_STREAM("ID: " << sonar.id << " - D0: " << sonar.distance[0] <<
        //	               ", D1: " << sonar.distance[1]);
        
        // Average but not dividing by ten to convert cm into mm
        if (sonar.id == 112){
            for (int j=0;j<10;j++)
            {dsx1=dsx1+sonar.distance[j];}
        }
        if (sonar.id == 115){
            for (int j=0;j<10;j++)
            {dsy1=dsy1+sonar.distance[j];}
        }
        if (sonar.id == 124){
            for (int j=0;j<10;j++)
            {dsz3=dsz3+sonar.distance[j];}
	    if(dsz3>hmax){dsz3=0;}
	    else {++okdsz;}}
        
        if (sonar.id == 125){
            for (int j=0;j<10;j++)
            {dsz4=dsz4+sonar.distance[j];}
            if(dsz4>hmax){dsz4=0;} 
            else {++okdsz;}}
        
        if (sonar.id == 126){
            for (int j=0;j<10;j++)
            {dsz1=dsz1+sonar.distance[j];}
            if(dsz1>hmax){dsz1=0;} 
            else {++okdsz;}}
        
        if (sonar.id == 127){
            for (int j=0;j<10;j++)
            {dsz2=dsz2+sonar.distance[j];}
            if(dsz2>hmax){dsz2=0;} 
            else {++okdsz;}}
        
    }
    if(okdsz!=0){dszt[4]=(dsz1+dsz2+dsz3+dsz4)/okdsz;}
    else {dszt[4]=dszt[3];}
    //ROS_INFO("distance 1: %f, distance 2: %f, distance x : %f, distance y : %f",dsz1,dsz2,dsx1,dsy//
}

void subComp(const sensors::compass::ConstPtr& msg)
{
    //ROS_INFO_STREAM("ID: " << msg->id << " - RZ0: " << msg->rz[0] <<
    //                ", RZ1: " << msg->rz[1]);
	
    if (msg->id == 96){
	rz1=(msg->rz[0])-rz0;}
    if (start && rz1!=0){rz0=rz1;
	start=false;}
    
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
  dsx2=ps.pose.position.x;
  dsy2=ps.pose.position.y;
  
  	    //Convert from angle to Quaterion
	    //Eigen::Quaterniond orientation = Eigen::Quaterniond(Eigen::AngleAxisd(cubesAngles[0], Eigen::Vector3d::UnitZ()));
	    //Eigen::AngleAxisf ang = Eigen::Quaternionf(ps.pose.orientation);
	    
  //rz2=-sgn(ps.pose.orientation.z)*ps.pose.orientation.w;
  Eigen::Quaterniond quad(ps.pose.orientation.x,ps.pose.orientation.y,ps.pose.orientation.z,ps.pose.orientation.w);
  rz2 = atan2(quad.toRotationMatrix()(1, 2), quad.toRotationMatrix()(2, 2));
  if(rz2 != rz2) //Testing if NaN
    rz2=0;
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
	//gethostname(rosname,100);
	sprintf(rosname,"state_estimator_%s",get_ip());
	
	ros::init(argc, argv, rosname);
    	ros::NodeHandle node;
    	ros::Publisher Controle_node = node.advertise<state::state>("/192_168_10_241/state",1);
    	ros::Rate loop_rate(10);
    	geometry_msgs::Pose pose;
    	state::state state;
	int print=0;
   double time[5]={0,1,2,3,4};

	//ros::Subscriber subA = node.subscribe("/android/imu", 1, poseCallback);
	ros::Subscriber subS = node.subscribe("/192_168_10_241/sonars", 1, subSonar);
	ros::Subscriber subC = node.subscribe("/192_168_10_241/compass",1,subComp);
	ros::Subscriber subI = node.subscribe("/192_168_10_241/imubuf",1,subImu);
	ros::Subscriber subSick = node.subscribe("/cubeA_pose", 1, poseCallback);
	while (ros::ok())
	{
        	////////////////////////////////////
        	////       State estimator      ////
        	////////////////////////////////////
		dsztf[4]=3.159*dsztf[3]-3.815*dsztf[2]+2.076*dsztf[1]-0.4291*dsztf[0]+0.01223*dszt[4]-0.02416*dszt[3]+0.03202*dszt[2]-0.02416*dszt[1]+0.01223*dszt[0];
		double avgz=(dsztf[4]+dsztf[3]+dsztf[2]+dsztf[1]+dsztf[0])/5;
		double avgt=(time[4]+time[3]+time[2]+time[1]+time[0])/(10*5);
		double Sxy=0;
		double Sx=0;
		for(int i=0;i<5;i++){
		    Sxy=(time[i]-avgt)*(dsztf[i]-avgz);
		    Sx=(time[i]-avgt)*(time[i]-avgt);
		}
		
        	state.pos[0]=dsx2;//dsx1;
        	state.pos[1]=dsy2;//dsy1;
        	state.pos[2]=dsztf[4]/1000;
        	state.quat[0]=rz2;//rz1
        	state.quat[1]=0;
        	state.quat[2]=0;
        	state.quat[3]=0;
		state.vel[0]=0;
		state.vel[1]=0;
		state.vel[2]=0;
		state.angvel[0]=0;
                state.angvel[1]=0;
                state.angvel[2]=0;
             	/////////////////////////////////
        	if(print==5){ROS_INFO("dist z(raw): %f, z(filtered): %f, dist x : %f, dist y : %f, rz : %f,z1:%f,z2:%f,z3:%f,z4:%f",dszt[4],dsztf[4]/1000,dsx2,dsy2,rz2,dsz1,dsz2,dsz3,dsz4);
		print=0;}
		else {print++;}
	        Controle_node.publish(state);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
