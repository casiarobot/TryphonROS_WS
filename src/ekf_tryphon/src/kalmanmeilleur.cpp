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

// Global variable
ros::Time beginTime;

double x=0;
double y=0;
double z=0;


double tx=0;
double ty=0;
double tz=0;


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
double offRoll =0;
double offPitch =0;

// Sonars
const int sonarNb = 2;
std::vector<float> sonarValues(sonarNb);
std::vector<bool> sonarActive(sonarNb);
float gainIIR;


Eigen::Quaterniond quatIMU(0.99255, 0,0.12187, 0); // quat of the rotation matrix between the tryphon frame and the IMU frame
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
    RCoeffs[0] = config.Leddar;
    RCoeffs[1] = config.Compass;
    RCoeffs[2] = config.IMU;
    RCoeffs[3] = config.ArTag;
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
    if(start[nb])
    {
        start[nb] = true;
        ROS_INFO("Sensor %i started", nb);
    }
    msgReceived[nb] = true;
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

    updateSensorInfo(0);
    double dsx = 0;
    double dsy = 0;
    double dsz = 0;

        for (int i=0; i < msg->leddars.size(); ++i)
        {
                const sensors::leddar &leddar = msg->leddars[i];
                //ROS_INFO_STREAM("ID: " << sonar.id << " - D0: " << sonar.distance[0] <<
                //	               ", D1: " << sonar.distance[1]);

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
    measures[0] << 30.0-dsz;
    z=dsz;
    measureTimes[0] = ros::Time::now();
}

void subComp(const sensors::compass::ConstPtr& msg)
{
    //ROS_INFO("Comp");
    updateSensorInfo(1);
        if(msg->id == (int)(0xC0)/2)
        {
        tz = msg->rz[0]/180.0*M_PI;
        measures[1] << updateAngleCount(msg->rz[0]/180.0*M_PI, measures[1][0], countYaw);
        measureTimes[1] = ros::Time::now();

    }
}

void subImu(const sensors::imubuff Imu)
{
    //ROS_INFO("Imu");
    updateSensorInfo(2);
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
    measures[2] << updateAngleCount(roll-offRoll, measures[2][0], countRoll), updateAngleCount(pitch-offPitch, measures[2][1], countPitch); //, avel_temp;
    //ROS_INFO("IMU: %f, %f, %f, %f, %f",roll, pitch, Imu_msg.linear_acceleration.x, Imu_msg.linear_acceleration.y, Imu_msg.linear_acceleration.z);
    measureTimes[2] = ros::Time::now();
}

void subLeddar2(const leddartech::leddar_data msg)  // sensor 0
{
    updateSensorInfo(3);
    x = msg.distance;
    measures[3] << 30.0-msg.distance;
    measureTimes[3] = ros::Time::now();
}

void subArTag(const ar_track_alvar_msgs::AlvarMarkers Aposes)
{
    updateSensorInfo(3);
    ar_track_alvar_msgs::AlvarMarker alvar;
    alvar=Aposes.markers[0];
    geometry_msgs::Pose Pose1=alvar.pose.pose ;

    measures[3] << Pose1.position.x, Pose1.position.y;//, Pose1.position.z;
    //measureTimes[3] = time(NULL);
}

geometry_msgs::PoseStamped pose2ros(Eigen::VectorXd pose)
{
    geometry_msgs::PoseStamped Pose;
    Pose.header.stamp=ros::Time::now();
    Pose.pose.position.x=pose(0);
    Pose.pose.position.y=pose(1);
    Pose.pose.position.z=pose(2);

    Pose.pose.orientation.x=pose(3);
    Pose.pose.orientation.y=pose(4);
    Pose.pose.orientation.z=pose(5);

    return Pose;
}

geometry_msgs::TwistStamped vel2ros(Eigen::VectorXd vel)
{
      geometry_msgs::TwistStamped Twist;
      Twist.header.stamp=ros::Time::now();
      Twist.twist.linear.x=vel(0);
      Twist.twist.linear.y=vel(1);
      Twist.twist.linear.z=vel(2);

      Twist.twist.angular.x=vel(3);
      Twist.twist.angular.y=vel(4);
      Twist.twist.angular.z=vel(5);
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
    ros::Subscriber subS = n.subscribe("/192_168_10_243/sonars", 1, subSonar);
    ros::Subscriber subL = n.subscribe("/192_168_10_242/leddars", 1, subLeddar);
    ros::Subscriber subC = n.subscribe("/192_168_10_242/compass",1, subComp);
    ros::Subscriber subI = n.subscribe("/192_168_10_243/imubuff",1, subImu);
    ros::Subscriber subL2 = n.subscribe("/leddar_one", 1, subLeddar2);
    //ros::Subscriber subAr = n.subscribe("/192_168_10_242/artags/artag1/ar_pose_marker",1, subArTag);

      //Publishers //
    ros::Publisher pubP = n.advertise<geometry_msgs::PoseStamped>("state_estimator/pose",1);
    ros::Publisher pubPraw = n.advertise<geometry_msgs::PoseStamped>("state_estimator/rawpose",1);
    ros::Publisher pubV = n.advertise<geometry_msgs::TwistStamped>("state_estimator/vel",1);
    ros::Publisher pubS = n.advertise<sensor_msgs::Range>("state_estimator/sonars",1);


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
    kalmanbasefilter kalLeddarX(stateSize);
    kalLeddarX.init(stateSize, Eigen::VectorXd::Zero(3*stateSize), 5*Eigen::MatrixXd::Identity(3*stateSize,3*stateSize), difRosTime(ros::Time::now(),beginTime));

    stateSize = 1;
    kalmanbasefilter kalCompass(stateSize);
    kalCompass.init(stateSize, Eigen::VectorXd::Zero(3*stateSize), 5*Eigen::MatrixXd::Identity(3*stateSize,3*stateSize), difRosTime(ros::Time::now(),beginTime));

    stateSize = 2;
    kalmanbasefilter kalIMU(stateSize);
    kalIMU.init(stateSize, Eigen::VectorXd::Zero(3*stateSize), 5*Eigen::MatrixXd::Identity(3*stateSize,3*stateSize), difRosTime(ros::Time::now(),beginTime));

    stateSize = 1;
    kalmanbasefilter kalLeddarZ(stateSize);
    kalLeddarZ.init(stateSize, Eigen::VectorXd::Zero(3*stateSize), 5*Eigen::MatrixXd::Identity(3*stateSize,3*stateSize), difRosTime(ros::Time::now(),beginTime));


    // Define H and Q matrices for sensors
    // leddar
    Eigen::MatrixXd H1(1,3);
    H1 << Eigen::MatrixXd::Identity(1,1), Eigen::MatrixXd::Zero(1,2);
    kalLeddarZ.set_measHandR(H1,RCoeffs[0]*Eigen::MatrixXd::Identity(1,1), 0);

    measures[0].resize(1);

    // compass
    Eigen::MatrixXd H2(1,3);
    H2 << Eigen::MatrixXd::Identity(1,1), Eigen::MatrixXd::Zero(1,2);
    kalCompass.set_measHandR(H2,RCoeffs[1]*Eigen::MatrixXd::Identity(1,1), 0);
    measures[1].resize(1);

    // IMU
    Eigen::MatrixXd H3(2,6);
    H3 << Eigen::MatrixXd::Identity(2,2),Eigen::MatrixXd::Zero(2,4);
            //Eigen::MatrixXd::Zero(3,9), Eigen::MatrixXd::Identity(3,3), Eigen::MatrixXd::Zero(3,6);
    kalIMU.set_measHandR(H3,RCoeffs[2]*Eigen::MatrixXd::Identity(2,2), 0);
    measures[2].resize(2);

    // leddar 2
    Eigen::MatrixXd H4(1,3);
    H4 << Eigen::MatrixXd::Identity(1,1), Eigen::MatrixXd::Zero(1,2);
    kalLeddarX.set_measHandR(H4,RCoeffs[3]*Eigen::MatrixXd::Identity(1,1), 0);
    measures[3].resize(1);

    ROS_INFO("Ready to filter");

    // AR tag

    /*Eigen::MatrixXd H4(2,18);
    H4 << Eigen::MatrixXd::Identity(2,2), Eigen::MatrixXd::Zero(2,16);
    kal.set_measHandR(H4,RCoeffs[3]*Eigen::MatrixXd::Identity(2,2), 3);

    measures[3].resize(2);*/
    Eigen::VectorXd state;
    state.resize(12);

    while (ros::ok())
    {
        if(reconfigure){
            updateRmatrices(kalLeddarX, 0);
            updateRmatrices(kalCompass, 1);
            updateRmatrices(kalIMU, 2);
            updateRmatrices(kalLeddarZ, 3);
            reconfigure =false;
        }


        for(int i=0; i<sensorNb; i++ ) // Kalman doing its stuff
        {
            if(msgReceived[i])
            {
                ROS_INFO("Sensor : %i", i);
                if(i==0)
                {
                        Eigen::Vector2d vals = kalLeddarZ.update(measures[i], 0, difRosTime(ros::Time::now(),beginTime)).head(2);
                        ROS_INFO("Sensor : %i", i);
                        state[2] = vals[0];
                        ROS_INFO("Sensor : %i", i);
                        state[8] = vals[1];
                        ROS_INFO("ok1");
                }
                if(i==1)
                {
                    ROS_INFO("ok2");
                        Eigen::Vector2d vals = kalCompass.update(measures[i], 0, difRosTime(ros::Time::now(),beginTime)).head(2);
                        state[5] = vals[0];
                        state[11] = vals[1];
                }
                if(i==2)
                {
                    ROS_INFO("ok3");
                        Eigen::Vector4d vals = kalIMU.update(measures[i], 0, difRosTime(ros::Time::now(),beginTime)).head(4);
                        state[3] = vals[0];
                        state[4] = vals[1];
                        state[9] = vals[2];
                        state[10] = vals[3];
                }
                if(i==3)
                {
                    ROS_INFO("ok4");
                        Eigen::Vector2d vals = kalLeddarX.update(measures[i], 0, difRosTime(ros::Time::now(),beginTime)).head(2);
                        state[0] = vals[0];
                        state[6] = vals[1];
                 }

                geometry_msgs::PoseStamped Pose = pose2ros(state.head(6));

                Eigen::VectorXd rawEigPose = Eigen::VectorXd::Zero(6);
                //rawEigPose << measures[3](0,0) +2.0, measures[0], measures[2], measures[1];
                rawEigPose << x +2.0, y, z, tx, ty, tz;
                geometry_msgs::PoseStamped rawPose = pose2ros(rawEigPose);

                geometry_msgs::TwistStamped Twist = vel2ros(state.segment(6,6));
                pubP.publish(Pose);
                pubPraw.publish(rawPose);
                pubV.publish(Twist);
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
        sensor_msgs::Range sMsg;
        sMsg.header.stamp = ros::Time::now();
        sMsg.max_range = sonarValues[0];
        sMsg.min_range = sonarValues[1];
        sMsg.range = funky(sonarActive);

        ros::spinOnce();
        loop_rate.sleep();
    }


  return 0;
}
