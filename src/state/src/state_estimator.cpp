// Standard C/C++ libraries
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <ctime>
#include <iostream>
#include <Eigen/Dense>
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
#include "geometry_msgs/Pose.h"
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
double dsz1=0;
double dsz2=0;
double dsz3=0;
double dsz4=0;
double dszt[5]={0,0,0,0,0};
double dsztf[5]={0,0,0,0,0};
//double dszt=0;
//double dszto=0;
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
double x[5]={0,0,0,0,0};
double y[5]={0,0,0,0,0};
double z[5]={0,0,0,0,0};
double xf[5]={0,0,0,0,0};
double yf[5]={0,0,0,0,0};
double zf[5]={0,0,0,0,0};

double q0[5]={0,0,0,0,0};
double q1[5]={0,0,0,0,0};
double q2[5]={0,0,0,0,0};
double q3[5]={0,0,0,0,0};
double q0f[5]={0,0,0,0,0};
double q1f[5]={0,0,0,0,0};
double q2f[5]={0,0,0,0,0};
double q3f[5]={0,0,0,0,0};

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
	rz=(msg->rz[0])-rz0;}
    if (start && rz!=0){rz0=rz;
	start=false;}
    
    //ROS_INFO("rotation: %f",rz); 
}

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

void subMCPTAM(const geometry_msgs::PoseArray Aposes)
{
    geometry_msgs::Pose pose=Aposes.poses[0];
    for(int i=0; i<4;i++){
        x[i]=x[i+1];
        xf[i]=xf[i+1];
        y[i]=y[i+1];
        yf[i]=yf[i+1];
        z[i]=z[i+1];
        zf[i]=zf[i+1];

        q0[i]=q0[i+1];
        q0f[i]=q0f[i+1];
        q1[i]=q1[i+1];
        q1f[i]=q1f[i+1];
        q2[i]=q2[i+1];
        q2f[i]=q2f[i+1];
        q3[i]=q3[i+1];
        q3f[i]=q3f[i+1];

    }
    x[4]=pose.position.x;
    y[4]=pose.position.y;
    z[4]=pose.position.z;

    q1[4]=pose.orientation.x;
    q2[4]=pose.orientation.y;
    q3[4]=pose.orientation.z;
    q0[4]=pose.orientation.z;
    

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
    /*sprintf(rosname,"/%s/sonars",temp_arg.c_str());
    ros::Subscriber subS = node.subscribe(rosname, 1, subSonar);
    sprintf(rosname,"/%s/compass",temp_arg.c_str());
    ros::Subscriber subC = node.subscribe(rosname,1,subComp);
    sprintf(rosname,"/%s/imbuf",temp_arg.c_str());
    ros::Subscriber subI = node.subscribe(rosname,1,subImu);*/
    //ros::Subscriber subSick = node.subscribe("/cubeA_pose", 1, poseCallback);
    ros::Subscriber subM = node.subscribe("mcptam/tracker_pose_array",1,subMCPTAM);


    double temps[5]={0,1.0/10.0,2.0/10.0,3.0/10.0,4.0/10.0};
    double avgt,avgx,avgy,avgz,St,Stx,Sty,Stz;
    double theta=110.0/180.0*M_PI;
    Eigen::Matrix3d Rmatrix;
    ROS_INFO("theta : %f",theta);
    Rmatrix<< 1, 0, 0,
              0, cos(theta), -sin(theta),
              0, sin(theta), cos(theta);
    std::cout<<Rmatrix<<std::endl;
    Eigen::Vector3d poscamf, posglobf, orientcamf, orientglobf, velcamf, velglobf, avelcamf, avelglobf;
    while (ros::ok())
    {
        ////////////////////////////////////
        ////       State estimator      ////
        ////////////////////////////////////
        dsztf[4]=3.159*dsztf[3]-3.815*dsztf[2]+2.076*dsztf[1]-0.4291*dsztf[0]+0.01223*dszt[4]-0.02416*dszt[3]+0.03202*dszt[2]-0.02416*dszt[1]+0.01223*dszt[0];
        xf[4]=3.159*xf[3]-3.815*xf[2]+2.076*xf[1]-0.4291*xf[0]+0.01223*x[4]-0.02416*x[3]+0.03202*x[2]-0.02416*x[1]+0.01223*x[0];
        yf[4]=3.159*yf[3]-3.815*yf[2]+2.076*yf[1]-0.4291*yf[0]+0.01223*y[4]-0.02416*y[3]+0.03202*y[2]-0.02416*y[1]+0.01223*y[0];
        zf[4]=3.159*zf[3]-3.815*zf[2]+2.076*zf[1]-0.4291*zf[0]+0.01223*z[4]-0.02416*z[3]+0.03202*z[2]-0.02416*z[1]+0.01223*z[0];

        q0f[4]=3.159*q0f[3]-3.815*q0f[2]+2.076*q0f[1]-0.4291*q0f[0]+0.01223*q0[4]-0.02416*q0[3]+0.03202*q0[2]-0.02416*q0[1]+0.01223*q0[0];
        q1f[4]=3.159*q1f[3]-3.815*q1f[2]+2.076*q1f[1]-0.4291*q1f[0]+0.01223*q1[4]-0.02416*q1[3]+0.03202*q1[2]-0.02416*q1[1]+0.01223*q1[0];
        q2f[4]=3.159*q2f[3]-3.815*q2f[2]+2.076*q2f[1]-0.4291*q2f[0]+0.01223*q2[4]-0.02416*q2[3]+0.03202*q2[2]-0.02416*q2[1]+0.01223*q2[0];
        q3f[4]=3.159*q3f[3]-3.815*q3f[2]+2.076*q3f[1]-0.4291*q3f[0]+0.01223*q3[4]-0.02416*q3[3]+0.03202*q3[2]-0.02416*q3[1]+0.01223*q3[0];

        //avgz=(dsztf[4]+dsztf[3]+dsztf[2]+dsztf[1]+dsztf[0])/5;
        avgx=(xf[4]+xf[3]+xf[2]+xf[1]+xf[0])/5.0;
        avgy=(yf[4]+yf[3]+yf[2]+yf[1]+yf[0])/5.0;
        avgz=(zf[4]+zf[3]+zf[2]+zf[1]+zf[0])/5.0;
        avgt=(temps[4]+temps[3]+temps[2]+temps[1]+temps[0])/(5.0);
        Stx=0;
        Sty=0;
        Stz=0;
        St=0;
        for(int i=0;i<5;i++){
            Stx+=(temps[i]-avgt)*(xf[i]-avgx);
            Sty+=(temps[i]-avgt)*(yf[i]-avgy);
            Stz+=(temps[i]-avgt)*(zf[i]-avgz);
            //Stz=(temps[i]-avgt)*(dsztf[i]-avgz);
            St+=(temps[i]-avgt)*(temps[i]-avgt);
        }


        /////////////////////////////////////

        ////////////////////////////////////
        ////          Rotation          ////
        ////////////////////////////////////   

        poscamf(0)=xf[4]; 
        poscamf(1)=yf[4];   
        poscamf(2)=zf[4];

        orientcamf(0)=q1f[4];
        orientcamf(1)=q2f[4];
        orientcamf(2)=q3f[4];

        velcamf(0)=Stx/St;
        velcamf(1)=Sty/St;   
        velcamf(2)=Stz/St;

        posglobf=Rmatrix*poscamf;
        velglobf=Rmatrix*velcamf;
        orientglobf=Rmatrix*orientcamf;



        /////////////////////////////////////
        state.pos[0]=posglobf(0);
        state.pos[1]=posglobf(1);
        state.pos[2]=posglobf(2);//dsztf[4];;
        state.quat[0]=q0f[4];
        state.quat[1]=orientglobf(0);
        state.quat[2]=orientglobf(1);
        state.quat[3]=orientglobf(2);
        state.vel[0]=velglobf(0);
        state.vel[1]=velglobf(1);
        state.vel[2]=velglobf(2);
        state.angvel[0]=0;
        state.angvel[1]=0;
        state.angvel[2]=0;
             	/////////////////////////////////
        

        ROS_INFO("x : %f , y: %f, z : %f, q0 : %f, q1 : %f,q2 :%f, q3 :%f",posglobf(0),posglobf(1),posglobf(2),q0[4],state.quat[1],state.quat[2],state.quat[3]);
        ROS_INFO("vx : %f , vy: %f, vz : %f, aq0 : %f, aq1 : %f,aq2 :%f, aq3 :%f",velglobf(0),velglobf(1),velglobf(2),q0f[4],state.quat[1],state.quat[2],state.quat[3]);
        //if(print==0){ROS_INFO("dist z(raw): %f, z(filtered): %f, dist x : %f, dist y : %f, rz : %f,z1:%f,z2:%f,z3:%f,z4:%f",dszt[4],dsztf[4],dsx1,dsy1,rz,dsz1,dsz2,dsz3,dsz4);
        //    print=0;}
        //else {++print;}
        Controle_node.publish(state);
        ros::spinOnce();
        loop_rate.sleep();
    }
return 0;
}
