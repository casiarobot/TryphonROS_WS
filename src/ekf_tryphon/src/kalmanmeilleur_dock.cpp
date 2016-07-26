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
#include <limits>

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

// Global variable
ros::Time beginTime;

double x=0;
double y=0;
double z=0;

double biaswx=0;
double biaswy=0;
double biaswz=0;

double tx=0;
double ty=0;
double tz=0;
int imu_counter=0;

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
double offRoll =3.14;
double offPitch =-.863;

bool gyro_bias;
double biaswx1,biaswy1,biaswz1,biaswxavg[300],biaswyavg[300],biaswzavg[300];


// Sonars
const int sonarNb = 2;
std::vector<float> sonarValues(sonarNb);
std::vector<bool> sonarActive(sonarNb);
float gainIIR;

//ARtags
Eigen::Vector3d ALVAR1position1, ALVAR1position2,ALVAR2position1, ALVAR2position2,ALVARangle;
Eigen::Quaterniond quatposAR(.7071,-.7071,0,0); //to make x y z of camera/artag output alligned with tryphon body frame 
//Eigen::Quaterniond quat15degyawAR( 0.9962,0,0,-.0872);
//Eigen::Quaterniond quattempAR=quat15degyawAR*quatposAR;
Eigen::Quaterniond quattempAR=quatposAR; //no camera angle
Eigen::Matrix3d RmatrixAR=quattempAR.toRotationMatrix();
geometry_msgs::PoseStamped Poseforrviz,IMU_out;

//For Real Tryphom
Eigen::Quaterniond quatIMU(0.9914,0.0000,-0.1305,0.0000); //for 242 in dock setup

//Eigen::Quaterniond quatIMU(0.99255, 0,0.12187, 0); // quat of the rotation matrix between the tryphon frame and the IMU frame
//For Gazebo
//Eigen::Quaterniond quatIMU(1, 0,0, 0); // quat of the rotation matrix between the tryphon frame and the IMU frame

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

biaswx=config.biaswx;
biaswy=config.biaswy;
biaswz=config.biaswz;
gyro_bias=config.bias_check;


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
//for compass
/*
void subComp(const sensors::compass::ConstPtr& msg) //in gazebo relative yaw from compas is given (ideally), t.b.d in real tryphon
{
       double  yaw_desired=187/180.0*M_PI; //for real tryphon
    //ROS_INFO("Comp");
    updateSensorInfo(0);
        if(msg->id == (int)(0xC0)/2)
        {
        tz=yaw_desired-msg->rz[0]/180.0*M_PI;
        measures[0] << tz ;//updateAngleCount(msg->rz[0]/180.0*M_PI, measures[0][0], countYaw);
        measureTimes[0] = ros::Time::now();

    }
}
*/
//for artag hack
void subComp(const geometry_msgs::PoseStamped compasspose) //in gazebo relative yaw from compas is given (ideally), t.b.d in real tryphon
{
       double  yaw_desired=0.0/180.0*M_PI; //for real tryphon
    //ROS_INFO("Comp");
    updateSensorInfo(0);
       
        tz=compasspose.pose.orientation.z-yaw_desired;///180.0*M_PI;
        measures[0] << tz ;//updateAngleCount(msg->rz[0]/180.0*M_PI, measures[0][0], countYaw);
        measureTimes[0] = ros::Time::now();

 
}


void subImu(const sensors::imubuff Imu)  //might have to find a way to deal with offset
{
    if(gyro_bias)
    {
        if(imu_counter<300)
            {imu_counter+=1;}

            for (int j1=1;j1<imu_counter;j1++)
            {
                biaswx1=biaswx1+biaswxavg[j1];
                biaswy1=biaswy1+biaswyavg[j1];
                biaswz1=biaswz1+biaswzavg[j1];
            }
                biaswx1=biaswx1/(double)imu_counter;
                biaswy1=biaswy1/(double)imu_counter;
                biaswz1=biaswz1/(double)imu_counter;
        
        ROS_INFO("biaswx1: %f,biaswy1: %f,biaswz1: %f",biaswx1,biaswy1,biaswz1);
    }


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
    avel_temp(0)=Imu_msg.angular_velocity.x-biaswx; // defined in IMU frame
    avel_temp(1)=Imu_msg.angular_velocity.y-biaswy;
    avel_temp(2)=Imu_msg.angular_velocity.z-biaswz;
    avel_temp=RIMUmatrix*avel_temp;  // defined in body frame
    tx = roll;
    ty = pitch;
   // avel_temp=EulerU::RbodyEuler(roll,pitch)*avel_temp; //I think this is transforming from angular vel to euler rates which is incorrect
    measures[1] << updateAngleCount(roll-offRoll, measures[1][0], countRoll), updateAngleCount(pitch-offPitch, measures[1][1], countPitch),avel_temp(0),avel_temp(1),avel_temp(2); //, avel_temp; - the bias of gyro
    //ROS_INFO("IMU: %f, %f, %f, %f, %f",roll, pitch, Imu_msg.linear_acceleration.x, Imu_msg.linear_acceleration.y, Imu_msg.linear_acceleration.z);
    measureTimes[1] = ros::Time::now();

}



void subArTag1(const geometry_msgs::PoseStamped Pose1)
{
    updateSensorInfo(2);   
    measures[2] <<  Pose1.pose.position.x,  Pose1.pose.position.y,  Pose1.pose.position.z; //or some distance between and also in target frame so not quite right!!
 measureTimes[2] = ros::Time::now();
}

//will need to be fixed
void subArTag2(const  geometry_msgs::PoseStamped Pose1)
{
    updateSensorInfo(3);
    measures[3] <<  Pose1.pose.position.x,  Pose1.pose.position.y,  Pose1.pose.position.z; //or some distance between and also in target frame so not quite right!!
 measureTimes[3] = ros::Time::now();
}


//will need to be fixed
//old artag replaced with artemp
/*
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
            
            if(Aposes1.markers[c].id==2 && !id_2){alvartemp1=Aposes1.markers[c];Pose2=alvartemp1.pose.pose;id_2=true;}
            else if (Aposes1.markers[c].id==3 && !id_3){alvartemp2=Aposes1.markers[c];Pose1=alvartemp2.pose.pose;id_3=true;}
        }

        if(id_2 && id_3){both=true;}
    }
    else if(Aposes1.markers.size()==0){ROS_INFO("no tag seen 1");return;}
  

if(id_3)
{

  ALVAR1position1(0)=Pose1.position.x; //  switch direction of vector
  ALVAR1position1(1)=Pose1.position.y;
  ALVAR1position1(2)=Pose1.position.z;
ALVAR1position1=RmatrixAR*ALVAR1position1;
}

if(id_2)
{
  ALVAR1position2(0)=Pose2.position.x; //  switch direction of vector
  ALVAR1position2(1)=Pose2.position.y;
  ALVAR1position2(2)=Pose2.position.z;
ALVAR1position2=RmatrixAR*ALVAR1position2;
}


else
{
ALVAR1position2=ALVAR1position2;
}

//ROS_INFO("angle= %f",angletemp1(1));
if(id_2 && !both)
{

measures[2] << ALVAR1position2(0), ALVAR1position2(1), ALVAR1position2(2)+.1;
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
    measures[3] <<  ALVAR2position2(0),  ALVAR2position2(1),  ALVAR2position2(2)+.1; //or some distance between and also in target frame so not quite right!!
}
else 
{
    measures[3] << ALVAR2position1(0), ALVAR2position1(1), ALVAR2position1(2);
} 

}
*/


//For Gazebo
/*

void subComp(const sensors::compass::ConstPtr& msg) //in gazebo relative yaw from compas is given (ideally), t.b.d in real tryphon
{
    //ROS_INFO("Comp");
    updateSensorInfo(0);
        if(msg->id == (int)(0xC0)/2)
        {
        tz = msg->rz[0]/180.0*M_PI;
        measures[0] << updateAngleCount(msg->rz[0]/180.0*M_PI, measures[0][0], countYaw);
        measureTimes[0] = ros::Time::now();

    }
}

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

void subImu(const sensor_msgs::Imu Imu_msg)
{

updateSensorInfo(1);
  double roll, pitch;
  EulerU::getRollPitchIMU(Imu_msg,roll,pitch);
 // angle(0)=(angle(0)+roll)/2; // low pass filter
 // angle(1)=(angle(1)-0.244+pitch)/2; // low pass filter

  Eigen::Vector3d avel_temp;
  avel_temp(0)=Imu_msg.angular_velocity.x; // defined in IMU frame
  avel_temp(1)=Imu_msg.angular_velocity.y;
  avel_temp(2)=Imu_msg.angular_velocity.z;
  avel_temp=RIMUmatrix*avel_temp;  // defined in body frame

  //avel_temp=EulerU::RbodyEuler(angle(0),angle(1))*avel_temp;

 
    tx = roll;
    ty = pitch;
    avel_temp=EulerU::RbodyEuler(roll,pitch)*avel_temp;
    measures[1] << updateAngleCount(roll-offRoll, measures[1][0], countRoll), updateAngleCount(pitch-offPitch, measures[1][1], countPitch), avel_temp; //, avel_temp;
    //ROS_INFO("IMU: %f, %f, %f, %f, %f",roll, pitch, Imu_msg.linear_acceleration.x, Imu_msg.linear_acceleration.y, Imu_msg.linear_acceleration.z);
    measureTimes[1] = ros::Time::now();

IMU_out.pose.position.x=roll; //rpy
IMU_out.pose.position.y=pitch;
IMU_out.pose.orientation.x=avel_temp(0);  //avel
IMU_out.pose.orientation.y=avel_temp(1); 
IMU_out.pose.orientation.z=avel_temp(2); 


}


void subArTag1(const geometry_msgs::PoseStamped Pose1)
{
    updateSensorInfo(2);   
Poseforrviz.pose=Pose1.pose;

    measures[2] <<  Pose1.pose.position.x+GaussNoise(),  Pose1.pose.position.y+GaussNoise(),  Pose1.pose.position.z+GaussNoise(); //or some distance between and also in target frame so not quite right!!
 measureTimes[2] = ros::Time::now();
}

//will need to be fixed
void subArTag2(const  geometry_msgs::PoseStamped Pose1)
{
    updateSensorInfo(3);
   
//haven't included a way to exclude measurements where tag dissapears


    measures[3] <<  Pose1.pose.position.x+GaussNoise(),  Pose1.pose.position.y+GaussNoise(),  Pose1.pose.position.z+GaussNoise(); //or some distance between and also in target frame so not quite right!!
 measureTimes[3] = ros::Time::now();
}


///////////////////////////end of gazebo

*/

/*
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
*/

void subLeddar2(const leddartech::leddar_data msg)  // sensor 0
{
    updateSensorInfo(5);
    x = msg.distance;
    measures[5] << 30.0-msg.distance;
    measureTimes[5] = ros::Time::now();
}

geometry_msgs::PoseStamped pose2ros(Eigen::VectorXd pose1, Eigen::VectorXd pose2)
{
    geometry_msgs::PoseStamped Pose;
    Pose.header.stamp=ros::Time::now();
    Pose.pose.position.x=pose1(0);
    Pose.pose.position.y=pose1(1);
    Pose.pose.position.z=pose1(2);

    Pose.pose.orientation.x=pose2(0);
    Pose.pose.orientation.y=pose2(1);
    Pose.pose.orientation.z=pose2(2);

    return Pose;
}

geometry_msgs::TwistStamped vel2ros(Eigen::VectorXd vel1,Eigen::VectorXd vel2)
{
      geometry_msgs::TwistStamped Twist;
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
    ros::init(argc, argv, "kalmanmeilleur_dock");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");
    //  Get Params
    nh.getParam("offRoll", offRoll);
    nh.getParam("offPitch", offPitch);
    ROS_INFO("Roll offset: %f, Pitch offset: %f", offRoll, offPitch);


    //ros::Subscriber subM = nh.subscribe("mcptam/tracker_pose_array",1,subMCPTAM);
   // ros::Subscriber subS = n.subscribe("/192_168_10_243/sonars", 1, subSonar);
   // ros::Subscriber subL = n.subscribe("/192_168_10_243/leddars", 1, subLeddar);
    //ros::Subscriber subC = n.subscribe("/192_168_10_242/compass",1, subComp);
       ros::Subscriber subC = n.subscribe("/192_168_10_242/compasscheat",1, subComp);
//For Real Tryphon
  
     ros::Subscriber subAr1 = n.subscribe("/192_168_10_241/artemp_pose1",1, subArTag1);
    ros::Subscriber subAr2 = n.subscribe("/192_168_10_242/artemp_pose2",1, subArTag2);
        ros::Subscriber subI = n.subscribe("/192_168_10_242/imubuff",1, subImu);
    

    //For Gazebo
    /*
    ros::Subscriber subAr1 = n.subscribe("/192_168_10_243/artags1/artag/ar_pose_marker",1, subArTag1);
   ros::Subscriber subAr2 = n.subscribe("/192_168_10_243/artags2/artag/ar_pose_marker",1, subArTag2);
     ros::Subscriber subI = n.subscribe("/192_168_10_243/raw_imu",1, subImu);
*/
    



    //ros::Subscriber subL2 = n.subscribe("/leddar_one", 1, subLeddar2);
    

      //Publishers //
    ros::Publisher pubAR1 = n.advertise<geometry_msgs::PoseStamped>("/192_168_10_241/ar_pose1",1);
    ros::Publisher pubAR2 = n.advertise<geometry_msgs::PoseStamped>("/192_168_10_241/ar_pose2",1);
    ros::Publisher pubvel1 = n.advertise<geometry_msgs::TwistStamped>("/192_168_10_241/ar_vel1",1);
    ros::Publisher pubvel2 = n.advertise<geometry_msgs::TwistStamped>("/192_168_10_241/ar_vel2",1);
    //ros::Publisher pubS = n.advertise<sensor_msgs::Range>("state_estimator/sonars",1);
    ros::Publisher pubPraw = n.advertise<geometry_msgs::PoseStamped>("state_estimator/rawpose",1);
   ros::Publisher pubIMUout = n.advertise<geometry_msgs::PoseStamped>("/IMU_out",1);
    // tf
    static tf::TransformBroadcaster br;
    tf::Transform transform;

    // Dynamic reconfigure
    dynamic_reconfigure::Server<ekf_tryphon::kalmanfilterConfig> server;
    dynamic_reconfigure::Server<ekf_tryphon::kalmanfilterConfig>::CallbackType f;
    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    ros::Rate loop_rate(100);

    //Init values
    beginTime = ros::Time::now();
    RIMUmatrix=quatIMU.toRotationMatrix(); // need to be computed only once

    // Init Kalmans
   
    int stateSize = 1;
    kalmanbasefilter kalCompass(stateSize);
    kalCompass.init(stateSize, Eigen::VectorXd::Zero(3*stateSize), 5*Eigen::MatrixXd::Identity(3*stateSize,3*stateSize), difRosTime(ros::Time::now(),beginTime));

    stateSize = 5; //including gyro
    kalmanbasefilter kalIMU(stateSize);
    kalIMU.init(stateSize, Eigen::VectorXd::Zero(3*stateSize), 5*Eigen::MatrixXd::Identity(3*stateSize,3*stateSize), difRosTime(ros::Time::now(),beginTime));


    stateSize = 3;
    kalmanbasefilter kalARtag1(stateSize);
    kalARtag1.init(stateSize, Eigen::VectorXd::Zero(3*stateSize), 5*Eigen::MatrixXd::Identity(3*stateSize,3*stateSize), difRosTime(ros::Time::now(),beginTime));

    stateSize = 3;
    kalmanbasefilter kalARtag2(stateSize);
    kalARtag2.init(stateSize, Eigen::VectorXd::Zero(3*stateSize), 5*Eigen::MatrixXd::Identity(3*stateSize,3*stateSize), difRosTime(ros::Time::now(),beginTime));

   stateSize = 1;
    kalmanbasefilter kalLeddarX(stateSize);
    kalLeddarX.init(stateSize, Eigen::VectorXd::Zero(3*stateSize), 5*Eigen::MatrixXd::Identity(3*stateSize,3*stateSize), difRosTime(ros::Time::now(),beginTime));


    stateSize = 1;
    kalmanbasefilter kalLeddarZ(stateSize);
    kalLeddarZ.init(stateSize, Eigen::VectorXd::Zero(3*stateSize), 5*Eigen::MatrixXd::Identity(3*stateSize,3*stateSize), difRosTime(ros::Time::now(),beginTime));


    // Define H and Q matrices for sensors

    // compass
    Eigen::MatrixXd H1(1,3);
    H1 << Eigen::MatrixXd::Identity(1,1), Eigen::MatrixXd::Zero(1,2);
    kalCompass.set_measHandR(H1,RCoeffs[0]*Eigen::MatrixXd::Identity(1,1), 0);
    measures[0].resize(1);

    // IMU
    Eigen::MatrixXd H2(5,15);
    H2 << Eigen::MatrixXd::Identity(5,5),Eigen::MatrixXd::Zero(5,10);
            //Eigen::MatrixXd::Zero(3,9), Eigen::MatrixXd::Identity(3,3), Eigen::MatrixXd::Zero(3,6);
    kalIMU.set_measHandR(H2,RCoeffs[1]*Eigen::MatrixXd::Identity(5,5), 0);
    measures[1].resize(5);

 // AR tag

    Eigen::MatrixXd H3(3,9);
    H3 << Eigen::MatrixXd::Identity(3,3), Eigen::MatrixXd::Zero(3,6);
    kalARtag1.set_measHandR(H3,RCoeffs[2]*Eigen::MatrixXd::Identity(3,3), 0);
    measures[2].resize(3);
     // AR tag



   Eigen::MatrixXd H4(3,9);
    H4 << Eigen::MatrixXd::Identity(3,3), Eigen::MatrixXd::Zero(3,6);
    kalARtag2.set_measHandR(H4,RCoeffs[2]*Eigen::MatrixXd::Identity(3,3), 0);

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

        pubIMUout.publish(IMU_out);
        if(reconfigure){
            updateRmatrices(kalCompass, 0);
            updateRmatrices(kalIMU,1);
            updateRmatrices(kalARtag1,2);
             updateRmatrices(kalARtag2,3);
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
                        Eigen::VectorXd vals = kalCompass.update(measures[i], 0, difRosTime(ros::Time::now(),beginTime)).head(2);
                        state[8] = vals[0];
                        state[17] = vals[1]; //this is yaw dot, we need w_z from imu section
            
                }
                if(i==1)
                {
                    ROS_INFO("ok3");
                        Eigen::VectorXd vals = kalIMU.update(measures[i], 0, difRosTime(ros::Time::now(),beginTime)).head(5);
                        state[6] = vals[0];
                        state[7] = vals[1];
                        state[15] = vals[2];
                        state[16] = vals[3];
                        state[17] = vals[4];
                }
                if(i==2)
                {
                        Eigen::VectorXd vals = kalARtag1.update(measures[i], 0, difRosTime(ros::Time::now(),beginTime)).head(6);
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
                         //state[8]=asin((state[4]-state[1])/1.95);;//of main tag (-) for proper orientation (+) y cameras   ///(-) removed for -y camera

                }
                if(i==3)
                {
                        Eigen::VectorXd vals = kalARtag2.update(measures[i], 0, difRosTime(ros::Time::now(),beginTime)).head(6);
                         ROS_INFO("Sensor : %i", i);
                        state[3] = vals[0];
                        state[4] = vals[1];
                        state[5] = vals[2];
                        ROS_INFO("Sensor : %i", i);
                        state[12] = vals[3];
                        state[13] = vals[4];
                        state[14] = vals[5];
                        ROS_INFO("ok1");
                        //state[8]=asin((state[4]-state[1])/1.95);//of main tag (-) for proper orientation (+) y cameras   ///(-) removed for -y camer
                }

                if(i==4)
                {
                        Eigen::Vector2d vals = kalLeddarZ.update(measures[i], 0, difRosTime(ros::Time::now(),beginTime)).head(2);
                        ROS_INFO("Sensor : %i", i);
                        state[3] = vals[0];
                        ROS_INFO("Sensor : %i", i);
                       // state[8] = vals[1];
                        ROS_INFO("ok1");
                }
                   if(i==5)
                {
                    ROS_INFO("ok4");
                        Eigen::Vector2d vals = kalLeddarX.update(measures[i], 0, difRosTime(ros::Time::now(),beginTime)).head(2);
                        state[0] = vals[0];
                        state[6] = vals[1];
                 }


                geometry_msgs::PoseStamped Pose1 = pose2ros(state.segment(0,3),state.segment(6,3));
                geometry_msgs::PoseStamped Pose2 = pose2ros(state.segment(3,3),state.segment(6,3));
                Eigen::VectorXd rawEigPose = Eigen::VectorXd::Zero(6);
                
                rawEigPose << Poseforrviz.pose.position.x,Poseforrviz.pose.position.y,Poseforrviz.pose.position.z, measures[1].head(2), measures[0];
                //rawEigPose << x +2.0, y, z, tx, ty, tz;
                geometry_msgs::PoseStamped rawPose = pose2ros(rawEigPose.head(3),rawEigPose.segment(3,3));

                geometry_msgs::TwistStamped Twist1 = vel2ros(state.segment(9,3),state.segment(15,3));
                geometry_msgs::TwistStamped Twist2 = vel2ros(state.segment(12,3),state.segment(15,3));
                pubAR1.publish(Pose1);
                pubAR2.publish(Pose2);
                pubPraw.publish(rawPose);
                pubvel1.publish(Twist1);
                pubvel2.publish(Twist2);
                transform.setOrigin( tf::Vector3(state[0], state[1], state[2]) );
                tf::Quaternion q;
                q.setEulerZYX(state[8], state[7], state[6]);
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
