#ifndef KALMAN_UTILITY_H
#define KALMAN_UTILITY_H

#include <math.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace KalmanU{

  Eigen::MatrixXd Fmatrix(double Ts, int pose_size, bool accel)  // state size = pose size*2 if accel =false or pose size*3 if accel=wrong
  {
    Eigen::MatrixXd  I(pose_size,pose_size);
    I=Eigen::MatrixXd::Identity(pose_size,pose_size);
    Eigen::MatrixXd  O(pose_size,pose_size);
    O=Eigen::MatrixXd::Zero(pose_size,pose_size);
    if(accel)
    {
      Eigen::MatrixXd  F(3*pose_size,3*pose_size);
      F << I,Ts*I,Ts*Ts/2*I,
            O,I,Ts*I,
            O,O,I;
      return F;
    }
    else
    {
      Eigen::MatrixXd  F(2*pose_size,2*pose_size);
      F << I,Ts*I,
            O,I;
      return F;

    }
  }

  Eigen::MatrixXd Qmatrix(double Ts, int pose_size, bool accel)  // state size = pose size*2 if accel =false or pose size*3 if accel=wrong
  {
    Eigen::MatrixXd  I(pose_size,pose_size);
    I=Eigen::MatrixXd::Identity(pose_size,pose_size);
    Eigen::MatrixXd  O(pose_size,pose_size);
    O=Eigen::MatrixXd::Zero(pose_size,pose_size);
    if(accel)
    {
      Eigen::MatrixXd  Q(3*pose_size,3*pose_size);
      Q << I*pow (Ts,5)/20,I*pow (Ts,4)/8,I*pow (Ts,3)/6,
            I*pow (Ts,4)/8,I*pow (Ts,3)/3,I*pow (Ts,2)/2,
            I*pow (Ts,3)/6,I*pow (Ts,2)/2,I*Ts;
      return Q;
    }
    else
    {
      Eigen::MatrixXd  Q(2*pose_size,2*pose_size);
      /*Q << I*pow (Ts,5)/20,I*pow (Ts,4)/8,
            I*pow (Ts,4)/8,I*pow (Ts,3)/3;*/
      Q << I*Ts*0.01,I*0,
           I*0,I*Ts*0.01;
      return Q;

    }
  }




} // end namespace euler

#endif
