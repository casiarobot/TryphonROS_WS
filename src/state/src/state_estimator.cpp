// Standard C/C++ libraries
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <ctime>

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

void subMCPTAM(const geometry_msgs::Pose pose)
{
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

    q0[4]=pose.orientation.x;
    q1[4]=pose.orientation.y;
    q2[4]=pose.orientation.z;
    q3[4]=pose.orientation.z;


}


int main(int argc, char **argv)
{
   ros::init(argc, argv, "state_estimator");
   ros::NodeHandle node;
   ros::Publisher Controle_node = node.advertise<state::state>("state",1);
   ros::Rate loop_rate(10);
   geometry_msgs::Pose pose;
   state::state state;
   int print=0;

	//ros::Subscriber subA = node.subscribe("/android/imu", 1, poseCallback);
   ros::Subscriber subS = node.subscribe("sonars", 1, subSonar);
   ros::Subscriber subC = node.subscribe("compass",1,subComp);
   //ros::Subscriber subI = node.subscribe("imu",1,subImu);
   ros::Subscriber subM = node.subscribe("mcptam_pose",1,subMCPTAM);
   double time[5]={0,1,2,3,4};


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


        /////////////////////////////////////
        /////////////////////////////////////



        state.pos[0]=xf[4];
        state.pos[1]=yf[4];
        state.pos[2]=dsztf[4];//dszt[4];
        state.quat[0]=q0f[4];
        state.quat[1]=q1f[4];
        state.quat[2]=q2f[4];
        state.quat[3]=q3f[4];
        state.vel[0]=0;
        state.vel[1]=0;
        state.vel[2]=Sxy/Sx;
        state.angvel[0]=0;
        state.angvel[1]=0;
        state.angvel[2]=0;
             	/////////////////////////////////
        if(print==0){ROS_INFO("dist z(raw): %f, z(filtered): %f, dist x : %f, dist y : %f, rz : %f,z1:%f,z2:%f,z3:%f,z4:%f",dszt[4],dsztf[4],dsx1,dsy1,rz,dsz1,dsz2,dsz3,dsz4);
            print=0;}
        else {++print;}
        Controle_node.publish(state);
        ros::spinOnce();
        loop_rate.sleep();
}
return 0;
}
