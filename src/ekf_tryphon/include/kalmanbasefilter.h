#ifndef KALMANBASEFILTER_H
#define KALMANBASEFILTER_H

// Eigen libraries
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <vector>
#include <iostream>

struct measureHR
{
    Eigen::MatrixXd H;
    Eigen::MatrixXd R;
};

class kalmanbasefilter
{
public:
    kalmanbasefilter();
    kalmanbasefilter(int sensorNb);
    void init(int sizeState, Eigen::VectorXd Xinit, Eigen::MatrixXd  Pinit, float t);
    void set_measHandR(Eigen::MatrixXd H, Eigen::MatrixXd R, int measNb);
    void set_measR(Eigen::MatrixXd R, int measNb);
    Eigen::MatrixXd get_measR(int measNb);
    Eigen::VectorXd update(Eigen::VectorXd meas, int measNb, float t);

private:
    bool initialized;

    int stateSize;
    double timeStamp;

    Eigen::VectorXd X;
    Eigen::MatrixXd P;
    std::vector<measureHR> measuresHR;
};

#endif // KALMANBASEFILTER_H
