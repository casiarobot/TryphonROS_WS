#include "kalmanbasefilter.h"

kalmanbasefilter::kalmanbasefilter()
{
    initialized = false;
    measuresHR.resize(1);
}

void kalmanbasefilter::set_numSens(int sensorNb)
{
    //initialized = false;
    measuresHR.resize(sensorNb);
}


void kalmanbasefilter::init(int sizeState, Eigen::VectorXd Xinit, Eigen::MatrixXd  Pinit, float t)
{
    stateSize = sizeState;
    X = Xinit;
    P = Pinit;
    timeStamp = t;
    initialized = true;
}

void kalmanbasefilter::set_measHandR(Eigen::MatrixXd H, Eigen::MatrixXd R, int measNb)
{
    measureHR m;
    m.H = H;
    m.R = R;
    measuresHR.at(measNb) = m;
}

void kalmanbasefilter::set_measR(Eigen::MatrixXd R, int measNb)
{
    measuresHR[measNb].R = R;
}

Eigen::MatrixXd kalmanbasefilter::get_measR(int measNb)
{
    return measuresHR[measNb].R;
}

Eigen::VectorXd kalmanbasefilter::update(Eigen::VectorXd meas, int measNb, float t)
{
    if(initialized)
    {

        double dt = t-timeStamp;
        double dt2 = fmax(t-timeStamp,0.05);
        timeStamp = t;
        // Defining matrices H, R, F and Q
        Eigen::MatrixXd H = measuresHR[measNb].H;
        Eigen::MatrixXd R = measuresHR[measNb].R;


        Eigen::MatrixXd  I = Eigen::MatrixXd::Identity(stateSize,stateSize);
        Eigen::MatrixXd  O = Eigen::MatrixXd::Zero(stateSize,stateSize);
        Eigen::MatrixXd  F(3*stateSize,3*stateSize);
        F << I,dt*I,dt*dt/2*I,
                O,I,dt*I,
                O,O,I;
        Eigen::MatrixXd  Q(3*stateSize,3*stateSize);
        dt =dt2;
        /*Q << I*pow (dt,5)/20,I*pow (dt,4)/8,I*pow (dt,3)/6,
             I*pow (dt,4)/8,I*pow (dt,3)/3,I*pow (dt,2)/2,
             I*pow (dt,3)/6,I*pow (dt,2)/2,I*dt;*/
        Q << I*pow (dt,5)/20,O*pow (dt,4)/8,O*pow (dt,3)/6,
             O*pow (dt,4)/8,I*pow (dt,3)/3,O*pow (dt,2)/2,
             O*pow (dt,3)/6,O*pow (dt,2)/2,I*dt;
        // Predict
        X = F*X;
        P = F*P*F.transpose() + Q; //3 sigma of this value
        //Update //
        Eigen::VectorXd Y = meas - H*X;
        Eigen::MatrixXd S = H*P*H.transpose()+R;
        Eigen::MatrixXd K = P*H.transpose()*S.inverse();
        X = X + K*Y;
        P = (Eigen::MatrixXd::Identity(3*stateSize,3*stateSize) - K*H)*P;
        std::cout<<"kalmanbasefilter"<<std::endl;
        return X;
    }
    else
    {
        std::cout<<"Not initialized"<<std::endl;
        return Eigen::Vector3d::Zero();
    }

}

double kalmanbasefilter::getP(int i, int j)
{
    double value=P(i,j);
    return value;
}
