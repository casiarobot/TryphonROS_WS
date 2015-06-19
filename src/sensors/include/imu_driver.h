#ifndef IMU_DRIVER_H
#define IMU_DRIVER_H

#include "ros/ros.h"
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>

//#include "sensors/imuros"
#include "sensors/imubuff.h"

#include <stdio.h>
#include <unistd.h>

#include "ahrs.h"
#include "robocomm.h"

class ImuDriver
{
public:
    ImuDriver(std::string port, bool useIMU);
    void init();
    void loop();
private:
    void createPublishers();
    void updateSensorData();
    void imuZeroCalibration();

    sensor_msgs::Imu generateImuMessage();
    geometry_msgs::PoseStamped generatePoseMessage();
    geometry_msgs::Quaternion orientationConvertToROSMessage();
    sensor_msgs::Imu generateMagMessage();

//    ROBOCOMM Robovero;
    ros::NodeHandle nodeHandle;

    ros::Publisher pubImu;
    ros::Publisher pubMag;
    ros::Publisher pubPose;
    static const int LOOP_RATE = 200;
    bool useIMU;


    imu::Vector<3> a, g, m;
    imu::Quaternion rotation;


    ros::Rate loopRate;
};

#endif // IMU_DRIVER_H
