#include "ros/ros.h"
#include "std_msgs/String.h"
#include <tf/transform_broadcaster.h>

// Standard C/C++ libraries
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <ctime>
#include <iostream>
#include "kalmanbasefilter.h"
#include <cmath>


// ROS messages libraries
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>

//libraries for the sonars and the compass
#include "sensors/sonar.h"
#include "sensors/sonarArray.h"
#include "sensors/leddar.h"
#include "sensors/leddarArray.h"
#include "sensors/compass.h"
#include "sensors/imubuff.h"
#include "sensors/imuros.h"
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <leddartech/leddar_data.h>

// Eigen libraries
#include <Eigen/Geometry>
#include <Eigen/Dense>

// Euler file
#include <Euler_utility.h>

// Kalman file
#include <Kalman_utility.h>

#include <sstream>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <ekf_tryphon/kalmanfilterConfig.h>


#include <fstream>
#include <iostream>
std::ofstream fileAR1;
std::ofstream fileAR2;
std::ofstream fileIMU;
std::ofstream fileCompass;

// Global variable
ros::Time beginTimeAR1,beginTimeAR2,beginTimeIMU,beginTimeCompass;

double x=0;
double y=0;
double z=0;


double tx=0;
double ty=0;
double tz=0;
int carlo=151;

// Sensors
const int sensorNb = 4;
bool start[sensorNb];
bool msgReceived[sensorNb];
std::vector<Eigen::VectorXd> measures(sensorNb);
std::vector<ros::Time> measureTimes(sensorNb);
bool reconfigure;
std::vector<float> RCoeffs(sensorNb);

// Treatment of roll pitch and yaw
int countRoll = 0;
int countPitch = 0;
int countYaw = 0;
int setAR1,old_setAR1=0;
int setAR2,old_setAR2=0;
int setCompass,old_setCompass=0;
int setIMU,old_setIMU=0;
char setstringAR1[5];
char setstringAR2[5];
char setstringCompass[5];
char setstringIMU[5];

double offRoll =0;
double offPitch =0;

// Sonars
const int sonarNb = 2;
std::vector<float> sonarValues(sonarNb);
std::vector<bool> sonarActive(sonarNb);
float gainIIR;

//ARtags
Eigen::Vector3d ALVAR1position1, ALVAR1position2,ALVAR2position1, ALVAR2position2;
Eigen::Quaterniond quatposAR(.7071,-.7071,0,0); //to make x y z of camera/artag output alligned with tryphon body frame 
Eigen::Quaterniond quat15degyawAR( 0.9962,0,0,-.0872);
Eigen::Quaterniond quattempAR=quat15degyawAR*quatposAR;
Eigen::Matrix3d RmatrixAR=quattempAR.toRotationMatrix();

//For Real Tryphom
//Eigen::Quaterniond quatIMU(0.99255, 0,0.12187, 0); // quat of the rotation matrix between the tryphon frame and the IMU frame
//For Gazebo
Eigen::Quaterniond quatIMU(1, 0,0, 0); // quat of the rotation matrix between the tryphon frame and the IMU frame

Eigen::Matrix3d RIMUmatrix;

float updateAngleCount(float an, float ao, int& count)
{
    float test = an + count*2*M_PI;
    if(ao-test > 1.80*M_PI){count += 1;}
    if(ao-test < -1.80*M_PI){count -= 1;}
    return an+count*2*M_PI;
}

float moduloFloat(float angle, float modulo)
{
    while(0<=angle && angle < modulo)
    {
        if(angle<0){angle +=modulo;}
        if(angle>modulo){angle -=modulo;}
    }
}

void callback(ekf_tryphon::kalmanfilterConfig &config, uint32_t level)
{
    ROS_INFO("Reconfigure Request");
    
    RCoeffs[0] = config.Compass;
    RCoeffs[1] = config.IMU;
    RCoeffs[2] = config.ArTag;
     RCoeffs[3] = config.Leddar;
    reconfigure = true;
}

inline float IIR(float old, float in, float gain)
{
    return old + (1.0-gain)*(in-old);
}

void initVar()
{
    reconfigure = false;
    for(int i=0; i<sensorNb; i++)
    {
        start[i] = false;
        msgReceived[i] = false;
    }
    for(int i=0; i<sonarNb; i++)
    {
        sonarValues[i] = 0;
        sonarActive[i] = false;
    }
    gainIIR = exp(-0.1/0.05);
}

void updateSensorInfo(int nb)
{
    if(!start[nb]) //i switched the ! becausde it was missing -Patrick
    {
        start[nb] = true;
        ROS_INFO("Sensor %i started", nb);
    }
    msgReceived[nb] = true;
}



//For Real tryphon
/*


void subComp(const sensors::compass::ConstPtr& msg) //in gazebo relative yaw from compas is given (ideally), t.b.d in real tryphon
{
       double  yaw_desired=292/180.0*M_PI; //for real tryphon
    //ROS_INFO("Comp");
    updateSensorInfo(0);
        if(msg->id == (int)(0xC0)/2)
        {
        tz=yaw_desired-msg->rz[0]/180.0*M_PI;
        measures[0] << tz ;//updateAngleCount(msg->rz[0]/180.0*M_PI, measures[0][0], countYaw);
        measureTimes[0] = ros::Time::now();

    }
}


void subImu(const sensors::imubuff Imu)  //might have to find a way to deal with offset
{
    //ROS_INFO("Imu");
    updateSensorInfo(1);
    sensor_msgs::Imu Imu_msg;
    float bufSize = Imu.buffer.size();
    for(int i=0; i<bufSize; i++)
    {
        Imu_msg.linear_acceleration.x += Imu.buffer[i].accel[0]/bufSize;
        Imu_msg.linear_acceleration.y += Imu.buffer[i].accel[1]/bufSize;
        Imu_msg.linear_acceleration.z += Imu.buffer[i].accel[2]/bufSize;

        Imu_msg.angular_velocity.x += Imu.buffer[i].gyro[0]/bufSize;
        Imu_msg.angular_velocity.y += Imu.buffer[i].gyro[1]/bufSize;
        Imu_msg.angular_velocity.z += Imu.buffer[i].gyro[2]/bufSize;
    }

    double roll, pitch;
    EulerU::getRollPitchIMU(Imu_msg,roll,pitch);

    Eigen::Vector3d avel_temp;
    avel_temp(0)=Imu_msg.angular_velocity.x; // defined in IMU frame
    avel_temp(1)=Imu_msg.angular_velocity.y;
    avel_temp(2)=Imu_msg.angular_velocity.z;
    avel_temp=RIMUmatrix*avel_temp;  // defined in body frame

    tx = roll;
    ty = pitch;
    avel_temp=EulerU::RbodyEuler(roll,pitch)*avel_temp;
    measures[1] << updateAngleCount(roll-offRoll, measures[1][0], countRoll), updateAngleCount(pitch-offPitch, measures[1][1], countPitch); //, avel_temp;
    //ROS_INFO("IMU: %f, %f, %f, %f, %f",roll, pitch, Imu_msg.linear_acceleration.x, Imu_msg.linear_acceleration.y, Imu_msg.linear_acceleration.z);
    measureTimes[1] = ros::Time::now();
}
//will need to be fixed


void subArTag1(const ar_track_alvar_msgs::AlvarMarkers Aposes1)
{
    updateSensorInfo(2);
   ar_track_alvar_msgs::AlvarMarker alvartemp1,alvartemp2;
    geometry_msgs::Pose Pose1 ;
    geometry_msgs::Pose Pose2  ;
    bool id_2=false,id_3=false,both=false,none=false; //none can be used in future to estimate next position possibly using velocity;   


RmatrixAR=quattempAR.toRotationMatrix();

    if(Aposes1.markers.size() > 0)
    {
        
        for (int c=0;c<Aposes1.markers.size();c++)
        {
            
            if(Aposes1.markers[c].id==2 && !id_2){alvartemp1=Aposes1.markers[c];Pose1=alvartemp1.pose.pose;id_2=true;}
            else if (Aposes1.markers[c].id==3 && !id_3){alvartemp2=Aposes1.markers[c];Pose2=alvartemp2.pose.pose;id_3=true;}
        }

        if(id_2 && id_3){both=true;}
    }
    else if(Aposes1.markers.size()==0){ROS_INFO("no tag seen 1");return;}
  

if(id_2)
{

  ALVAR1position1(0)=Pose1.position.x; //  switch direction of vector
  ALVAR1position1(1)=Pose1.position.y;
  ALVAR1position1(2)=Pose1.position.z;
ALVAR1position1=RmatrixAR*ALVAR1position1;
}

if(id_3)
{
  ALVAR1position2(0)=Pose2.position.x; //  switch direction of vector
  ALVAR1position2(1)=Pose2.position.y;
  ALVAR1position2(2)=Pose2.position.z;
ALVAR1position2=RmatrixAR*ALVAR1position2;
}

/*
else
{
ALVAR1position2=ALVAR1position2;
}

//ROS_INFO("angle= %f",angletemp1(1));
if(id_3 && !both)
{

measures[2] << ALVAR1position2(0), ALVAR1position2(1), ALVAR1position2(2)-.075;
}
else 
{
measures[2] << ALVAR1position1(0), ALVAR1position1(1), ALVAR1position1(2);
}
    //measureTimes[3] = time(NULL);
}

//will need to be fixed
void subArTag2(const ar_track_alvar_msgs::AlvarMarkers Aposes2)
{
    updateSensorInfo(3);
    
  ar_track_alvar_msgs::AlvarMarker alvartemp1,alvartemp2;
    geometry_msgs::Pose Pose1 ;
    geometry_msgs::Pose Pose2  ;
    bool id_0=false,id_1=false,both=false,none=false; //none can be used in future to estimate next position possibly using velocity;   

    if(Aposes2.markers.size() > 0)
    {
        
        for (int c=0;c<Aposes2.markers.size();c++)
        {
            
            if(Aposes2.markers[c].id==0 && !id_0){alvartemp1=Aposes2.markers[c];Pose1=alvartemp1.pose.pose;id_0=true; } //can change id's accordingly
            else if (Aposes2.markers[c].id==1 && !id_1){alvartemp2=Aposes2.markers[c];Pose2=alvartemp2.pose.pose;id_1=true;}
        }

        if(id_0 && id_1){both=true;}
    }
    else {ROS_INFO("no tag seen 2 ");return;}


//haven't included a way to exclude measurements where tag dissapears

if(id_0)
{
  ALVAR2position1(0)=Pose1.position.x; //  switch direction of vector
  ALVAR2position1(1)=Pose1.position.y;
  ALVAR2position1(2)=Pose1.position.z;
ALVAR2position1=RmatrixAR*ALVAR2position1;
}
if(id_1)
{
    
  ALVAR2position2(0)=Pose2.position.x; //  switch direction of vector
  ALVAR2position2(1)=Pose2.position.y;
  ALVAR2position2(2)=Pose2.position.z;
ALVAR2position2=RmatrixAR*ALVAR2position2;
}

if(id_1 && !both)
{
    measures[3] <<  ALVAR2position2(0),  ALVAR2position2(1),  ALVAR2position2(2)-.075; //or some distance between and also in target frame so not quite right!!
}
else 
{
    measures[3] << ALVAR2position1(0), ALVAR2position1(1), ALVAR2position1(2);
}

}
*/


//For Gazebo

double GaussNoise()
{
double mu=0.0;
double sigma=0.005;
srand(time(NULL));
    const double epsilon = std::numeric_limits<double>::min();


    static double z0, z1;
    static bool generate;
    generate = !generate;

    if (!generate)
       return z1 * sigma + mu;

    double u1, u2;
    do
     {
       u1 = rand() * (1.0 / RAND_MAX);
       u2 = rand() * (1.0 / RAND_MAX);
     }
    while ( u1 <= epsilon );

    z0 = sqrt(-2.0 * log(u1)) * cos(2*M_PI * u2);
    z1 = sqrt(-2.0 * log(u1)) * sin(2*M_PI* u2);
    //return 0;
   return z0 * sigma + mu;
}


void subComp(const geometry_msgs::PoseStamped msg) //in gazebo relative yaw from compas is given (ideally), t.b.d in real tryphon
{
    //ROS_INFO("Comp");
    updateSensorInfo(0);
     
        measures[0] <<  msg.pose.orientation.z;///180.0*M_PI;//updateAngleCount(msg.pose.orientation.z/180.0*M_PI, measures[0][0], countYaw);
        measureTimes[0] = ros::Time::now();

strcpy(setstringCompass,msg.header.frame_id.c_str());
setCompass=atoi(msg.header.frame_id.c_str());

if(old_setCompass!=setCompass)
{
beginTimeCompass=ros::Time::now();
}

old_setCompass=setCompass;

}

 void subImu(const geometry_msgs::PoseStamped Imu_msg)
{

updateSensorInfo(1);
 
 double roll=Imu_msg.pose.position.x;
 double pitch=Imu_msg.pose.position.y;
Eigen::Vector3d avel_temp;
 avel_temp(0)=Imu_msg.pose.orientation.x;
 avel_temp(1)=Imu_msg.pose.orientation.y;
 avel_temp(2)=Imu_msg.pose.orientation.z;
 

   
    measures[1] << roll,pitch,avel_temp; //, avel_temp;
    //ROS_INFO("IMU: %f, %f, %f, %f, %f",roll, pitch, Imu_msg.linear_acceleration.x, Imu_msg.linear_acceleration.y, Imu_msg.linear_acceleration.z);
    measureTimes[1] = ros::Time::now();

strcpy(setstringIMU,Imu_msg.header.frame_id.c_str());
setIMU=atoi(Imu_msg.header.frame_id.c_str());


if(old_setIMU!=setIMU)
{
beginTimeIMU=ros::Time::now();
}

old_setIMU=setIMU;



}


void subArTag1(const geometry_msgs::PoseStamped Pose1)
{
    updateSensorInfo(2);   
    measures[2] <<  Pose1.pose.position.x,  Pose1.pose.position.y,  Pose1.pose.position.z;
    //measures[2] <<  Pose1.pose.position.x+GaussNoise(),  Pose1.pose.position.y+GaussNoise(),  Pose1.pose.position.z+GaussNoise(); //or some distance between and also in target frame so not quite right!!
 measureTimes[2] = ros::Time::now();

strcpy(setstringAR1,Pose1.header.frame_id.c_str());
setAR1=atoi(Pose1.header.frame_id.c_str());

if(old_setAR1!=setAR1)
{
beginTimeAR1=ros::Time::now();
}

old_setAR1=setAR1;
}

//will need to be fixed
void subArTag2(const  geometry_msgs::PoseStamped Pose1)
{
    updateSensorInfo(3);
   
//haven't included a way to exclude measurements where tag dissapears

measures[3] <<  Pose1.pose.position.x,  Pose1.pose.position.y,  Pose1.pose.position.z;
    //measures[3] <<  Pose1.pose.position.x+GaussNoise(),  Pose1.pose.position.y+GaussNoise(),  Pose1.pose.position.z+GaussNoise(); //or some distance between and also in target frame so not quite right!!
 measureTimes[3] = ros::Time::now();

strcpy(setstringAR2,Pose1.header.frame_id.c_str());
setAR2=atoi(Pose1.header.frame_id.c_str());

if(old_setAR2!=setAR2)
{
beginTimeAR2=ros::Time::now();
}

old_setAR2=setAR2;
}





void subSonar(const sensors::sonarArray::ConstPtr& msg)
{
    //ROS_INFO("Sonars");

    for (int i=0; i<msg->sonars.size(); ++i)
    {
        const sensors::sonar &sonar = msg->sonars[i];
        if (sonar.id == (int)(0xE6)/2)
        {
            if(sonar.distance<600)
            {
                sonarValues[0] = IIR(sonarValues[0], sonar.distance, gainIIR); // Assuming that the sonar sampling is 10hz
                sonarActive[0] = true;
            }
            else{sonarActive[0] = false;}
        }
        else if (sonar.id == (int)(0xEA)/2){ sonarValues[1] = sonar.distance; }
        {
            if(sonar.distance<600)
            {
                sonarValues[1] = IIR(sonarValues[1], sonar.distance, gainIIR);
                sonarActive[1] = true;
            }
            else{sonarActive[1] = false;}
        }
    }
}

void subLeddar(const sensors::leddarArray::ConstPtr& msg)  // sensor 0
{

    updateSensorInfo(4);
    double dsx = 0;
    double dsy = 0;
    double dsz = 0;

        for (int i=0; i < msg->leddars.size(); ++i)
        {
                const sensors::leddar &leddar = msg->leddars[i];
                //ROS_INFO_STREAM("ID: " << sonar.id << " - D0: " << sonar.distance[0] <<
                //                 ", D1: " << sonar.distance[1]);

                // From cm to m
                switch (leddar.id) {
            case 0x92:
                                dsx = leddar.distance / 100.0;
                                break;
                        case 0x94:
                                dsy = leddar.distance / 100.0;
                                break;
                        case 0x98:
                                dsz = leddar.distance / 100.0;
                                break;

                }

        }
    measures[4] << 30.0-dsz;
    z=dsz;
    measureTimes[4] = ros::Time::now();
}


void subLeddar2(const leddartech::leddar_data msg)  // sensor 0
{
    updateSensorInfo(5);
    x = msg.distance;
    measures[5] << 30.0-msg.distance;
    measureTimes[5] = ros::Time::now();
}

geometry_msgs::PoseStamped pose2ros1(Eigen::VectorXd pose1,Eigen::VectorXd pose2)
{
    geometry_msgs::PoseStamped Pose;
    Pose.header.frame_id=setstringAR1;
    Pose.header.stamp=ros::Time::now();
    Pose.pose.position.x=pose1(0);
    Pose.pose.position.y=pose1(1);
    Pose.pose.position.z=pose1(2);

    Pose.pose.orientation.x=pose2(0);
    Pose.pose.orientation.y=pose2(1);
    Pose.pose.orientation.z=pose2(2);

    return Pose;
}
geometry_msgs::PoseStamped pose2ros2(Eigen::VectorXd pose1,Eigen::VectorXd pose2)
{
    geometry_msgs::PoseStamped Pose;
    Pose.header.frame_id=setstringAR2;
    Pose.header.stamp=ros::Time::now();
    Pose.pose.position.x=pose1(0);
    Pose.pose.position.y=pose1(1);
    Pose.pose.position.z=pose1(2);

    Pose.pose.orientation.x=pose2(0);
    Pose.pose.orientation.y=pose2(1);
    Pose.pose.orientation.z=pose2(2);

    return Pose;
}

geometry_msgs::TwistStamped vel2ros1(Eigen::VectorXd vel1,Eigen::VectorXd vel2)
{
      geometry_msgs::TwistStamped Twist;
      Twist.header.frame_id=setstringIMU;
      Twist.header.stamp=ros::Time::now();
      Twist.twist.linear.x=vel1(0);
      Twist.twist.linear.y=vel1(1);
      Twist.twist.linear.z=vel1(2);

      Twist.twist.angular.x=vel2(0);
      Twist.twist.angular.y=vel2(1);
      Twist.twist.angular.z=vel2(2);
      return Twist;
}

geometry_msgs::TwistStamped vel2ros2(Eigen::VectorXd vel1,Eigen::VectorXd vel2)
{
      geometry_msgs::TwistStamped Twist;
      Twist.header.frame_id=setstringCompass;
      Twist.header.stamp=ros::Time::now();
      Twist.twist.linear.x=vel1(0);
      Twist.twist.linear.y=vel1(1);
      Twist.twist.linear.z=vel1(2);

      Twist.twist.angular.x=vel2(0);
      Twist.twist.angular.y=vel2(1);
      Twist.twist.angular.z=vel2(2);
      return Twist;
}


void updateRmatrices(kalmanbasefilter& k, int nb)
{
    int n = k.get_measR(0).cols();
    k.set_measR(RCoeffs[nb]*Eigen::MatrixXd::Identity(n,n), 0);
}

float difRosTime(ros::Time t2, ros::Time t1)
{
    return t2.toSec()-t1.toSec();
}

float funky(std::vector<bool> sa)
{
    float v=0;
    for(int i=0; i<sonarNb; i++)
    {
        if(sonarActive[i])
            v += i+1;
    }
    return v;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ekf_tryphon");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");
    //  Get Params
    nh.getParam("offRoll", offRoll);
    nh.getParam("offPitch", offPitch);
    ROS_INFO("Roll offset: %f, Pitch offset: %f", offRoll, offPitch);

    //ros::Subscriber subM = nh.subscribe("mcptam/tracker_pose_array",1,subMCPTAM);
   // ros::Subscriber subS = n.subscribe("/192_168_10_243/sonars", 1, subSonar);
   // ros::Subscriber subL = n.subscribe("/192_168_10_243/leddars", 1, subLeddar);
    ros::Subscriber subC = n.subscribe("/192_168_10_243/compass",1, subComp);
     
//For Real Tryphon
     /*
     ros::Subscriber subAr1 = n.subscribe("/192_168_10_241/artemp_pose1",1, subArTag1);
    ros::Subscriber subAr2 = n.subscribe("/192_168_10_242/artemp_pose2",1, subArTag2);
    */

    //For Gazebo
    ros::Subscriber subAr1 = n.subscribe("/192_168_10_243/artags1/artag/ar_pose_marker",1, subArTag1);
   ros::Subscriber subAr2 = n.subscribe("/192_168_10_243/artags2/artag/ar_pose_marker",1, subArTag2);
    
    ros::Subscriber subI = n.subscribe("/192_168_10_243/raw_imu",1, subImu);
    
    //For Gazebo
   // ros::Subscriber subI = n.subscribe("/192_168_10_243/raw_imu",1, subImu);

    //ros::Subscriber subL2 = n.subscribe("/leddar_one", 1, subLeddar2);
    

      //Publishers //
    ros::Publisher pubAR1 = n.advertise<geometry_msgs::PoseStamped>("/192_168_10_241/ar_pose1",1);
    ros::Publisher pubAR2 = n.advertise<geometry_msgs::PoseStamped>("/192_168_10_241/ar_pose2",1);
    ros::Publisher pubvel1 = n.advertise<geometry_msgs::TwistStamped>("/192_168_10_241/ar_vel1",1);
    ros::Publisher pubvel2 = n.advertise<geometry_msgs::TwistStamped>("/192_168_10_241/ar_vel2",1);
    //ros::Publisher pubS = n.advertise<sensor_msgs::Range>("state_estimator/sonars",1);
    ros::Publisher pubPraw = n.advertise<geometry_msgs::PoseStamped>("state_estimator/rawpose",1);
    // tf
    static tf::TransformBroadcaster br;
    tf::Transform transform;


 fileAR1.open("AR1.csv");

fileAR1   << "posetime,poseset,x,y,z,vx,vy,vz,Px,Py,Pz,Pvx,Pvy,Pvz" << std::endl  ;
 
 fileAR2.open("AR2.csv");

fileAR2   << "posetime,poseset,x,y,z,vx,vy,vz,Px,Py,Pz,Pvx,Pvy,Pvz" << std::endl  ;



 fileIMU.open("IMU.csv");

fileIMU   << "posetime,poseset,roll,pitch,wx,wy,wz,Pr,Pp,Pwx,Pwy,Pwz" << std::endl  ;
 
 fileCompass.open("Compass.csv");

fileCompass   << "posetime,poseset,yaw,Py" << std::endl  ;

//filePAR.open("estimate_cov.csv");

//filePAR   << "time,AR1set,x1,y1,z1,AR2set,x2,y2,z2,IMUset,roll,pitch,Compassset,yaw,vx1,vy1,vz1,vx2,vy2,vz2,wx,wy,wz" << std::endl  ;




    // Dynamic reconfigure
    dynamic_reconfigure::Server<ekf_tryphon::kalmanfilterConfig> server;
    dynamic_reconfigure::Server<ekf_tryphon::kalmanfilterConfig>::CallbackType f;
    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    ros::Rate loop_rate(100);

    //Init values
    beginTimeAR1 = ros::Time::now();
      beginTimeAR2 = ros::Time::now();
        beginTimeIMU = ros::Time::now();
          beginTimeCompass = ros::Time::now();
    RIMUmatrix=quatIMU.toRotationMatrix(); // need to be computed only once

    // Init Kalmans

int stateSize = 1;
    Eigen::MatrixXd H1(1,3);
    H1 << Eigen::MatrixXd::Identity(1,1), Eigen::MatrixXd::Zero(1,2);
    kalmanbasefilter kalCompass[carlo];


    stateSize = 5; //including gyro  
    Eigen::MatrixXd H2(5,15);
    H2 << Eigen::MatrixXd::Identity(5,5),Eigen::MatrixXd::Zero(5,10);
    kalmanbasefilter kalIMU[carlo];

    stateSize = 3;
    Eigen::MatrixXd H3(3,9);
    H3 << Eigen::MatrixXd::Identity(3,3), Eigen::MatrixXd::Zero(3,6);
    kalmanbasefilter kalARtag1[carlo];

    stateSize = 3;
    Eigen::MatrixXd H4(3,9);
    H4 << Eigen::MatrixXd::Identity(3,3), Eigen::MatrixXd::Zero(3,6);
    kalmanbasefilter kalARtag2[carlo];

 

 for (int j=0;j<carlo;j++)
    {
stateSize = 1;
    kalCompass[j].set_numSens(stateSize);
    kalCompass[j].init(stateSize, Eigen::VectorXd::Zero(3*stateSize), 5*Eigen::MatrixXd::Identity(3*stateSize,3*stateSize), difRosTime(ros::Time::now(),beginTimeCompass));
kalCompass[j].set_measHandR(H1,RCoeffs[0]*Eigen::MatrixXd::Identity(1,1), 0);

    stateSize = 5;
     kalIMU[j].set_numSens(stateSize);
    kalIMU[j].init(stateSize, Eigen::VectorXd::Zero(3*stateSize), 5*Eigen::MatrixXd::Identity(3*stateSize,3*stateSize), difRosTime(ros::Time::now(),beginTimeIMU));
   kalIMU[j].set_measHandR(H2,RCoeffs[1]*Eigen::MatrixXd::Identity(5,5), 0);

     stateSize = 3;
      kalARtag1[j].set_numSens(stateSize);
    kalARtag1[j].init(stateSize, Eigen::VectorXd::Zero(3*stateSize), 5*Eigen::MatrixXd::Identity(3*stateSize,3*stateSize), difRosTime(ros::Time::now(),beginTimeAR1));
    kalARtag1[j].set_measHandR(H3,RCoeffs[2]*Eigen::MatrixXd::Identity(3,3), 0);

   stateSize = 3;
    kalARtag2[j].set_numSens(stateSize);
    kalARtag2[j].init(stateSize, Eigen::VectorXd::Zero(3*stateSize), 5*Eigen::MatrixXd::Identity(3*stateSize,3*stateSize), difRosTime(ros::Time::now(),beginTimeAR2));
   kalARtag2[j].set_measHandR(H4,RCoeffs[2]*Eigen::MatrixXd::Identity(3,3), 0);

    }

  


    // Define H and Q matrices for sensors

    // compass
    
    
    measures[0].resize(1);
    // IMU
            //Eigen::MatrixXd::Zero(3,9), Eigen::MatrixXd::Identity(3,3), Eigen::MatrixXd::Zero(3,6)
    measures[1].resize(5);
 // AR tag
    measures[2].resize(3);
     // AR ta
    measures[3].resize(3);
    

// leddar
/* 
   Eigen::MatrixXd H1(1,3);
    H5 << Eigen::MatrixXd::Identity(1,1), Eigen::MatrixXd::Zero(1,2);
    kalLeddarZ.set_measHandR(H5,RCoeffs[3]*Eigen::MatrixXd::Identity(1,1), 0);

    measures[0].resize(1);

    // leddar 2
    Eigen::MatrixXd H4(1,3);
    H6 << Eigen::MatrixXd::Identity(1,1), Eigen::MatrixXd::Zero(1,2);
    kalLeddarX.set_measHandR(H6,RCoeffs[3]*Eigen::MatrixXd::Identity(1,1), 0);
    measures[3].resize(1);
*/


    ROS_INFO("Ready to filter");


    Eigen::VectorXd state;
    state.resize(18); //bonus r1 r2 and dots

    while (ros::ok())
    {
        if(reconfigure){
            updateRmatrices(kalCompass[setCompass], 0);
            updateRmatrices(kalIMU[setIMU],1);
            updateRmatrices(kalARtag1[setAR1],2);
             updateRmatrices(kalARtag2[setAR2],2); //switched to 2 
            //updateRmatrices(kalLeddarZ, 3);
            //updateRmatrices(kalLeddarX, 4);
            reconfigure =false;
        }


        for(int i=0; i<sensorNb; i++ ) // Kalman doing its stuff
        {
            if(msgReceived[i])
            {
                ROS_INFO("Sensor : %i", i);
                if(i==0)
                {
                    ROS_INFO("ok2");
                        Eigen::Vector2d vals = kalCompass[setCompass].update(measures[i], 0, difRosTime(ros::Time::now(),beginTimeCompass)).head(2);
                        state[8] = vals[0];
                        //state[17] = vals[1]; //this is yaw dot, we need w_z from imu section
            
                }
                if(i==1)
                {
                    ROS_INFO("ok3");
                        Eigen::VectorXd vals = kalIMU[setIMU].update(measures[i], 0, difRosTime(ros::Time::now(),beginTimeIMU)).head(5);
                        state[6] = vals[0];
                        state[7] = vals[1];
                        state[15] = vals[2];
                        state[16] = vals[3];
                        state[17] = vals[4];
                }
                if(i==2)
                {
                        Eigen::VectorXd vals = kalARtag1[setAR1].update(measures[i], 0, difRosTime(ros::Time::now(),beginTimeAR1)).head(6);
                        ROS_INFO("Sensor : %i", i);
                        state[0] = vals[0];
                        state[1] = vals[1];
                          ROS_INFO("Sensor : %i", i);
                        state[2] = vals[2];
                        ROS_INFO("Sensor : %i", i);
                        state[9] = vals[3];
                        state[10] = vals[4];
                        state[11] = vals[5];
                        ROS_INFO("ok1");
                }
                if(i==3)
                {
                        Eigen::VectorXd vals = kalARtag2[setAR2].update(measures[i], 0, difRosTime(ros::Time::now(),beginTimeAR2)).head(6);
                         ROS_INFO("Sensor : %i", i);
                        state[3] = vals[0];
                        state[4] = vals[1];
                        state[5] = vals[2];
                        ROS_INFO("Sensor : %i", i);
                        state[12] = vals[3];
                        state[13] = vals[4];
                        state[14] = vals[5];
                        ROS_INFO("ok1");
                }

        
                geometry_msgs::PoseStamped Pose1 = pose2ros1(state.segment(0,3),state.segment(6,3));
                geometry_msgs::PoseStamped Pose2 = pose2ros2(state.segment(3,3),state.segment(6,3));
                Eigen::VectorXd rawEigPose = Eigen::VectorXd::Zero(6);
                
                //rawEigPose << measures[3](0,0) +2.0, measures[0], measures[2], measures[1];
                rawEigPose << x +2.0, y, z, tx, ty, tz;
                //geometry_msgs::PoseStamped rawPose = pose2ros(rawEigPose);

                geometry_msgs::TwistStamped Twist1 = vel2ros1(state.segment(9,3),state.segment(15,3));
                geometry_msgs::TwistStamped Twist2 = vel2ros2(state.segment(12,3),state.segment(15,3));
                pubAR1.publish(Pose1);
                pubAR2.publish(Pose2);
                //pubPraw.publish(rawPose);
                pubvel1.publish(Twist1);
                pubvel2.publish(Twist2);
                    
                fileAR1 <<difRosTime(ros::Time::now(),beginTimeAR1)<<","<<setstringAR1  << "," << Pose1.pose.position.x << ","<< Pose1.pose.position.y <<","<< Pose1.pose.position.z << "," <<Twist1.twist.linear.x << "," <<Twist1.twist.linear.y<< "," <<Twist1.twist.linear.z << "," << kalARtag1[setAR1].getP(0,0)<<","<<kalARtag1[setAR1].getP(1,1)<<","<<kalARtag1[setAR1].getP(2,2)<<","<<kalARtag1[setAR1].getP(3,3)<<","<<kalARtag1[setAR1].getP(4,4)<<","<<kalARtag1[setAR1].getP(5,5) <<std::endl;
                fileAR2 <<difRosTime(ros::Time::now(),beginTimeAR2)<<","<< setstringAR2 << "," << Pose2.pose.position.x << ","<< Pose2.pose.position.y <<","<< Pose2.pose.position.z <<","<<Twist2.twist.linear.x << "," <<Twist2.twist.linear.y<< "," <<Twist2.twist.linear.z << "," << kalARtag2[setAR2].getP(0,0)<<","<<kalARtag2[setAR2].getP(1,1)<<","<<kalARtag2[setAR2].getP(2,2)<<","<<kalARtag2[setAR2].getP(3,3)<<","<<kalARtag2[setAR2].getP(4,4)<<","<<kalARtag2[setAR2].getP(5,5) <<std::endl;
                fileIMU <<difRosTime(ros::Time::now(),beginTimeIMU)<<","<<setstringIMU<<","<< Pose1.pose.orientation.x <<","<< Pose1.pose.orientation.y <<","<< Twist1.twist.angular.x<<","<< Twist1.twist.angular.y<<","<< Twist1.twist.angular.z<<","<<kalIMU[setIMU].getP(0,0)<<","<<kalIMU[setIMU].getP(1,1)<<","<<kalIMU[setIMU].getP(2,2)<<","<<kalIMU[setIMU].getP(3,3)<<","<<kalIMU[setIMU].getP(4,4)<<std::endl; 
                fileCompass <<difRosTime(ros::Time::now(),beginTimeCompass)<<","<<setstringCompass<<","<< Pose1.pose.orientation.z <<","<<kalCompass[setCompass].getP(0,0) << std::endl; 


               // filePAR << ros::Time::now()<<","<< setstringAR1<<","<<kalARtag1[setAR1].getP(1,1)<<","<<kalARtag1[setAR1].getP(2,2)<<","<<kalARtag1[setAR1].getP(3,3)<<","<< setstringAR2<<","<<kalARtag2[setAR2].getP(1,1)<<","<<kalARtag2[setAR2].getP(2,2)<<","<<kalARtag2[setAR2].getP(3,3);
               // filePAR <<","<< setstringIMU<<","<<kalIMU[setIMU].getP(1,1)<<","<<kalIMU[setIMU].getP(2,2)<<","<< setstringCompass<<","<<kalCompass[setCompass].getP(1,1)<<","<<kalARtag1[setAR1].getP(4,4)<<","<<kalARtag1[setAR1].getP(5,5)<<","<<kalARtag1[setAR1].getP(6,6)<<","<<kalARtag2[setAR2].getP(4,4)<<","<<kalARtag2[setAR2].getP(5,5)<<","<<kalARtag2[setAR2].getP(6,6)<<","<<kalIMU[setIMU].getP(3,3)<<","<<kalIMU[setIMU].getP(4,4)<<","<<kalIMU[setIMU].getP(5,5)<<std::endl;

                transform.setOrigin( tf::Vector3(state[0], state[1], state[2]) );
                tf::Quaternion q;
                q.setEulerZYX(state[5], state[4], state[3]);
                transform.setRotation(q);
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "Pose_Filtered"));
                transform.setOrigin( tf::Vector3(rawEigPose[0], rawEigPose[1], rawEigPose[2]) );
                q.setEulerZYX(rawEigPose[5], rawEigPose[4], rawEigPose[3]);
                transform.setRotation(q);
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "Pose"));

                msgReceived[i]=false;
            }

        }
        //sensor_msgs::Range sMsg;
        //sMsg.header.stamp = ros::Time::now();
        //sMsg.max_range = sonarValues[0];
        //sMsg.min_range = sonarValues[1];
       // sMsg.range = funky(sonarActive);

        ros::spinOnce();
        loop_rate.sleep();
    }


  return 0;
}
