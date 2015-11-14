#ifndef EULER_UTILITY_H
#define EULER_UTILITY_H

#include <math.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <sensor_msgs/Imu.h>

namespace EulerU{

  void getRollPitchIMU(sensor_msgs::Imu Imu, double& roll, double& pitch)
  {
    double gx=Imu.linear_acceleration.x;
    double gy=Imu.linear_acceleration.y;
    double gz=Imu.linear_acceleration.z;

    roll = atan2(gy,gz);
    pitch = atan2(-gx,sqrt(gy*gy+gz*gz));
  }

  void getRollPitchIMU(double *accel, double& roll, double& pitch)
  {
    double gx=accel[0];
    double gy=accel[1];
    double gz=accel[2];

    roll = atan2(gy,gz);
    pitch = atan2(-gx,sqrt(gy*gy+gz*gz));
  }

  void getQuatFromEuler(Eigen::Quaterniond& quat, double roll, double pitch, double yaw)
  {
    Eigen::Quaterniond qx(cos(roll/2),sin(roll/2),0,0), qy(cos(pitch/2),0,sin(pitch/2),0), qz(cos(yaw/2),0,0,sin(yaw/2));
    quat= qx*qy*qz;
  }

  void getEulerFromQuat(Eigen::Quaterniond quat, double& roll, double& pitch, double& yaw)
  {
    roll=atan2(2*(quat.w()*quat.x()+quat.y()*quat.z()),1-2*(quat.x()*quat.x()+quat.y()*quat.y()));
    pitch=asin(2*(quat.w()*quat.y()-quat.z()*quat.x()));
    yaw=atan2(2*(quat.w()*quat.z()+quat.x()*quat.y()),1-2*(quat.z()*quat.z()+quat.y()*quat.y()));
  }

  Eigen::Matrix3d RbodyEuler(double roll,double pitch)
  {
    Eigen::Matrix3d R;
    double sinr=sin(roll);
    double sint=sin(pitch);
    double cosr=cos(roll);
    double cost=cos(pitch);
    double tant=tan(pitch);


    R<<1.0,sinr*tant,cosr*tant,
        0 ,cosr     ,-sinr    ,
        0 ,sinr/cost,cosr/cost;


    return R;
  }


} // end namespace euler

#endif
