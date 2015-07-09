//library for ros
#include <ros/ros.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include "controls/State.h"



#include <vector>
#include <cmath>


#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <dynamic_reconfigure/server.h>
#include <controls/trajectoryConfig.h>


#include "controls/Commands.h"

#include "vects2geoMsgs.cpp"

bool path=false;
int pathNb=0;
Eigen::Vector3d pos, angle, vel, avel;
Eigen::Vector3d posdesir, angledesir, veldesir, aveldesir, acceldesir, angleAcceldesir;
Eigen::Vector3d dAngle;
controls::State statedesir;



void callback(controls::trajectoryConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: x: %f, y: %f, z: %f, yaw: %f  ",
           config.xB,
           config.yB,
           config.zB,
           config.yawB);

  /*temps=config.temps;
  xB=config.xB;
  xE=config.xE;
  yB=config.yB;
  yE=config.yE;
  zB=config.zB;
  zE=config.zE;
  yawB=config.yawB;
  yawE=config.yawE;

  double dx = xE-xB;
  double dy = yE-yB;
  double dz = zE-zB;
  double dyaw = yawE-yawB;

  coefX={10/}
  */

}

void subPath(const geometry_msgs::Pose2D pose)
{
  path=pose.theta;
  pathNb=pose.x;
}

void subPose(const geometry_msgs::Pose pose)
{

    pos(0)=pose.position.x; // defined in global frame
    pos(1)=pose.position.y;
    pos(2)=pose.position.z;
    angle(0)=pose.orientation.x; // defined in IMU frame
    angle(1)=pose.orientation.y;
    angle(2)=pose.orientation.z;

}

void subVel(const geometry_msgs::TwistStamped velStamped)
{

	geometry_msgs::Twist vels=velStamped.twist;
    vel(0)=vels.linear.x; // defined in global frame
    vel(1)=vels.linear.y;
    vel(2)=vels.linear.z;
    avel(0)=vels.angular.x; // defined in IMU frame
    avel(1)=vels.angular.y;
    avel(2)=vels.angular.z;
}


void zero_vel()
{
  vect3_zero(veldesir);
  vect3_zero(aveldesir);
  vect3_zero(acceldesir);
  vect3_zero(angleAcceldesir);
}



double pPoly(double coef[3], double begin, double t)
{
  double dt=t-begin;
  return coef[0]*pow(dt,3)+coef[1]*pow(dt,4)+coef[2]*pow(dt,5);
}

double vPoly(double coef[3], double begin, double t)
{
  double dt=t-begin;
  return 3*coef[0]*pow(dt,2)+4*coef[1]*pow(dt,3)+5*coef[2]*pow(dt,4);
}

double aPoly(double coef[3], double begin, double t)
{
  double dt=t-begin;
  return 6*coef[0]*dt+12*coef[1]*pow(dt,2)+20*coef[2]*pow(dt,3);
}

void ComputeCoeffs(double (&c)[3], double dt, double dp)
{
  if(dt>0)
  {
  c[0]=10.0/pow(dt,3)*dp;
  c[1]=-15.0/pow(dt,4)*dp;
  c[2]=6.0/pow(dt,5)*dp;
  }
  else
  {
  c[0]=0;
  c[1]=0;
  c[2]=0;
  }

}

double modulo(double f) // not a modulo but dealing with the discontinuity around pi and -pi
{
  if(f>M_PI*(1+0.05)) // +0.05 to create an hysteresis
  {
   f=f-2*M_PI;
  }
  else
  {
    if(f<-M_PI*(1+0.05))
    {
      f=f+2*M_PI;
    }
  }
  return f;
}


int main(int argc, char **argv)
{

  std::string s;

  //std::replace(temp_arg.begin(), temp_arg.end(), '.', '_');
  //sprintf(rosname,"trajectory_%s",temp_arg.c_str());


  ros::init(argc, argv, "trajectory");
  ros::NodeHandle node;

  // Publishers //
  ros::Publisher State_pub = node.advertise<controls::State>("state_trajectory",1);
  ros::Publisher Path_pub = node.advertise<geometry_msgs::Pose2D>("path_info",1);



  // Subscribers //
  ros::Subscriber subP = node.subscribe("path_command", 1, subPath);
  ros::Subscriber subPo = node.subscribe("control/pose", 1, subPose);
  ros::Subscriber subV = node.subscribe("control/vel", 1, subVel);


  // Dynamic Reconfigure //
  dynamic_reconfigure::Server<controls::trajectoryConfig> server;
  dynamic_reconfigure::Server<controls::trajectoryConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  // Loop rate //
  ros::Rate loop_rate(10);

  //Variables //
  int step=0;
  geometry_msgs::Pose2D path_info;
  path_info.x=0; // path number
  path_info.y=0; // step if step there is
  path_info.theta=0; // onOff

  int maxThrust=100;
  bool noInt=false;

  bool path_debut=true;
  double path_debut_time=0;
  int ctrlNb=3;

  double GainCP=1;

  Eigen::Vector4d range(0.15,0.15,0.20,0.1);
  Eigen::Vector4d rangeV(0.2,0.2,0.2,0.2);



  //// PATHS ////
  // Defining paths //



  /*
  std::vector<Eigen::Vector4d> pathLine; // Gazebo paths
  Eigen::Vector4d p1(-4.5,0,0,0);
  Eigen::Vector4d p2(4.5,0,0,0);
  pathLine.push_back( p1);
  pathLine.push_back( p2);

  std::vector<Eigen::Vector4d> pathLine2;

  Eigen::Vector4d p12(4.5,0,0,M_PI);
  Eigen::Vector4d p22(-4.5,0,0,M_PI);

  pathLine2.push_back(p1);
  pathLine2.push_back(p2);
  pathLine2.push_back(p12);
  pathLine2.push_back(p22);

  std::vector<Eigen::Vector4d> pathSquare;
  Eigen::Vector4d pS(-3.5,-3.5,0,0);
  pathSquare.push_back(pS);

  pS(0)=3.5;
  pathSquare.push_back(pS);

  pS(1)=3.5;
  pathSquare.push_back(pS);

  pS(0)=-3.5;
  pathSquare.push_back(pS);*/

  std::vector<Eigen::Vector4d> pathLine;
  Eigen::Vector4d p1(-1.5,-1.5,2.5,M_PI/4);
  Eigen::Vector4d p2(1.5,1.5,2.5,M_PI/4);
  pathLine.push_back( p1);
  pathLine.push_back( p2);

  std::vector<Eigen::Vector4d> pathLine2;

  Eigen::Vector4d p12(1.5,1.5,2.5,-M_PI*3/4);
  Eigen::Vector4d p22(-1.5,-1.5,2.5,-M_PI*3/4);

  pathLine2.push_back(p1);
  pathLine2.push_back(p2);
  pathLine2.push_back(p12);
  pathLine2.push_back(p22);

  std::vector<Eigen::Vector4d> pathSquare;
  Eigen::Vector4d pS(-2.5,-2.5,2.5,0);
  pathSquare.push_back(pS);

  pS(0)=2.5;
  pathSquare.push_back(pS);

  pS(1)=2.5;
  pathSquare.push_back(pS);

  pS(0)=-2.5;
  pathSquare.push_back(pS);

  std::vector<Eigen::Vector4d> pathSquare2;


  std::vector< std::vector<Eigen::Vector4d> > paths;

  paths.push_back(pathLine);
  paths.push_back(pathLine2);
  paths.push_back(pathSquare);
  //paths.push_back(pathSquare2);

  // Rotation parameters //
  double omega=0.0333333333333333333333333333333;
  double r=2.5;
  double r1=2;
  double r2=0.5;
  double t=0;


  //////////////////////

  /// Definition of the coeffs for the specials paths ///
  double coeffs1[3]={0,0,0};
  double coeffs2[3]={0,0,0};
  double coeffs3[3]={0,0,0};
  double coeffs4[3]={0,0,0};
  double coeffs5[3]={0,0,0};
  double coeffs6[3]={0,0,0};
  double coeffs7[3]={0,0,0};
  double coeffs8x[3]={0,0,0};
  double coeffs8y[3]={0,0,0};
  double coeffs9[3]={0,0,0};
  double coeffs10[3]={0,0,0};
  double coeffs11[3]={0,0,0};

  ComputeCoeffs(coeffs1,30.00,6.0);
  ComputeCoeffs(coeffs2,16.0,1.6);
  ComputeCoeffs(coeffs3,15.0,2.0);
  ComputeCoeffs(coeffs4,20.0,1.6);
  ComputeCoeffs(coeffs5,20.0,1.0/4.0*M_PI);
  ComputeCoeffs(coeffs6,30.0,2.5);
  ComputeCoeffs(coeffs7,30.0,1.6);
  ComputeCoeffs(coeffs8x,60.0,6.5);
  ComputeCoeffs(coeffs8y,60.0,6.0);
  ComputeCoeffs(coeffs9,60.0,M_PI);
  ComputeCoeffs(coeffs10,25.0,2.5);
  ComputeCoeffs(coeffs11,25.0,3.0/4.0*M_PI);

  // Init //

  vect3_zero(posdesir);
  vect3_zero(angledesir);
  zero_vel();


  while(ros::ok())
  {
    ros::spinOnce();
    // Computation Dangle //
    // avoiding the discontunity due to the angle around pi and -pi //
    dAngle(0)=modulo(angle(0)-angledesir(0));
    dAngle(1)=modulo(angle(1)-angledesir(1));
    dAngle(2)=modulo(angle(2)-angledesir(2));
	ctrlNb=3;
    if(path)
    {
      if(path_debut)
        {
          path_debut_time=ros::Time::now().toSec();
          path_debut=false;
        }
        if(pathNb<3)
        {
          if(fabs(pos(0)-posdesir(0))<range(0) && fabs(pos(1)-posdesir(1))<range(1) && fabs(pos(2)-posdesir(2))<range(2) && fabs(dAngle(2))<range(3) && fabs(vel(0))<rangeV(0) && fabs(vel(1))<rangeV(1) && fabs(vel(2))<rangeV(2) && fabs(avel(2))<rangeV(3))
          {
            if(step<paths[pathNb].size()-1){step++;}
            else{step=0;}

          }
          posdesir(0)=paths[pathNb][step](0);
          posdesir(1)=paths[pathNb][step](1);
          posdesir(2)=paths[pathNb][step](2);
          angledesir(2)=paths[pathNb][step](3);
        }
        if(pathNb==3)
        {
          t= ros::Time::now().toSec() - path_debut_time;
          ROS_INFO("t= %f; omega= %f; sin= %f", t, omega,r*sin(omega*t) );


          posdesir(0)=r*sin(omega*t);
          posdesir(1)=r*cos(omega*t);
          posdesir(2)=2.5;

          veldesir(0)=omega*r*cos(omega*t);
          veldesir(1)=-omega*r*sin(omega*t);
          veldesir(2)=0;

          acceldesir(0)=-omega*omega*r*sin(omega*t);
          acceldesir(1)=-omega*omega*r*cos(omega*t);
          acceldesir(2)=0;
        }
        if(pathNb==4)
        {
          t= ros::Time::now().toSec() - path_debut_time;
          ROS_INFO("t= %f; omega= %f; sin= %f", t, omega,r*sin(omega*t) );


          /*posdesir(0)=r*sin(omega*t);
          posdesir(1)=r*cos(omega*t);
          posdesir(2)=2.5+r2*sin(omega*t);

          veldesir(0)=omega*r*cos(omega*t);
          veldesir(1)=-omega*r*sin(omega*t);
          veldesir(2)=omega*r2*cos(omega*t);

          acceldesir(0)=-omega*omega*r*sin(omega*t);
          acceldesir(1)=-omega*omega*r*cos(omega*t);
          acceldesir(2)=-omega*omega*r2*sin(omega*t);*/

          double value=0.7*t;

          while(value>M_PI)
          {
           value-=2*M_PI;
          }

          angledesir(2)=value;

        }
        if(pathNb==5)  // Attack (petit nom)
        {
          t= ros::Time::now().toSec() - path_debut_time;
          if(t<30) // go back
          {
            noInt=true;
            posdesir(0)=-pPoly(coeffs1,0,t);
            posdesir(1)=0;
            posdesir(2)=2.5;

            veldesir(0)=-vPoly(coeffs1,0,t);
            veldesir(1)=0;
            veldesir(2)=0;

            acceldesir(0)=-aPoly(coeffs1,0,t);
            acceldesir(1)=0;
            acceldesir(2)=0;
          }

          if(t>30 && t<40 ) // wait
          {
            vect3_zero(acceldesir);
            vect3_zero(angleAcceldesir);
            vect3_zero(veldesir);
            vect3_zero(aveldesir);
          }
          if(t>40 && t<56 ) // go up
          {
            posdesir(0)=-6;
            posdesir(1)=0;
            posdesir(2)=2.5+pPoly(coeffs2,40,t);

            veldesir(0)=0;
            veldesir(1)=0;
            veldesir(2)=vPoly(coeffs2,40,t);

            acceldesir(0)=0;
            acceldesir(1)=0;
            acceldesir(2)=aPoly(coeffs2,40,t);
          }

          if(t>56 && t<61 ) // wait
          {
            posdesir(0)=-6;
            posdesir(1)=0;
            posdesir(2)=4.1;

            vect3_zero(acceldesir);
            vect3_zero(angleAcceldesir);
            vect3_zero(veldesir);
            vect3_zero(aveldesir);
          }
          if(t>61 && t<101 ) // go down full speed and wait
          {
            posdesir(0)=2;
            posdesir(1)=0;
            posdesir(2)=2.5;
          }

          if(t>101 && t<116 ) // go back
          {
            noInt=false;
            posdesir(0)=2-pPoly(coeffs3,101,t);
            posdesir(1)=0;
            posdesir(2)=2.5;

            veldesir(0)=-vPoly(coeffs3,101,t);
            veldesir(1)=0;
            veldesir(2)=0;

            acceldesir(0)=-aPoly(coeffs3,101,t);
            acceldesir(1)=0;
            acceldesir(2)=0;
          }
          if(t>116 && t<180 ) // wait
          {
            vect3_zero(acceldesir);
            vect3_zero(angleAcceldesir);
            vect3_zero(veldesir);
            vect3_zero(aveldesir);
          }
          if(t>180)
          {
            pathNb+=1;
            path_debut_time=ros::Time::now().toSec();
          }



        }

        if(pathNb==6) // observator
        {
          t= ros::Time::now().toSec() - path_debut_time;
          if(t<36) // Three turns 3*2*M_PI/1
          {
            noInt=true;
            double value=0.348*t;

            while(value>M_PI)
            {
              value-=2*M_PI;
            }

            angledesir(2)=value;
            aveldesir(2)=0.348;

          }
          if(t>36 && t<38 ) // slow down
          {
            angledesir(2)=0;
            aveldesir(2)=0;
          }

          if(t>38 && t<58 ) // go up and turn
          {

            posdesir(0)=0;
            posdesir(1)=0;
            posdesir(2)=2.5+pPoly(coeffs4,38,t);

            veldesir(0)=0;
            veldesir(1)=0;
            veldesir(2)=vPoly(coeffs4,38,t);

            acceldesir(0)=0;
            acceldesir(1)=0;
            acceldesir(2)=aPoly(coeffs4,38,t);


            angledesir(2)=0.55*sin(0.17*(t-38));

            aveldesir(2)=0.17*0.55*cos(0.17*(t-38));

            angleAcceldesir(2)=-0.17*0.17*0.55*sin(12*(t-38));
          }
          if(t>58 && t<118) // turn -30 +30
          {
            posdesir(0)=0;
            posdesir(1)=0;
            posdesir(2)=4.1;

            veldesir(0)=0;
            veldesir(1)=0;
            veldesir(2)=0;

            angledesir(2)=0.55*sin(0.17*(t-38));

            aveldesir(2)=0.17*0.55*cos(0.17*(t-38));

            angleAcceldesir(2)=-0.17*0.17*0.55*sin(0.17*(t-38));
          }
          if(t>118 && t<180 ) // wait
          {
            noInt=false;
            angledesir(2)=0;
            vect3_zero(acceldesir);
            vect3_zero(angleAcceldesir);
            vect3_zero(veldesir);
            vect3_zero(aveldesir);
          }
          if(t>180)
          {
            pathNb+=1;
            path_debut_time=ros::Time::now().toSec();
          }

        }
        if(pathNb==7)
        {

          t= ros::Time::now().toSec() - path_debut_time;
          if(t>0 && t<20 ) // go down and in the corner and turn -1/4 pi
          {
            noInt=true;

            angledesir(2)=-pPoly(coeffs5,0,t);

            aveldesir(2)=-vPoly(coeffs5,0,t);

            angleAcceldesir(2)=-aPoly(coeffs5,0,t);
          }
          if(t>20 && t<50 ) // go down and in the corner
          {
            posdesir(0)=pPoly(coeffs6,20,t);
            posdesir(1)=pPoly(coeffs6,20,t);
            posdesir(2)=4.1-pPoly(coeffs7,20,t);

            veldesir(0)=vPoly(coeffs6,20,t);
            veldesir(1)=vPoly(coeffs6,20,t);
            veldesir(2)=-vPoly(coeffs7,20,t);

            acceldesir(0)=aPoly(coeffs6,20,t);
            acceldesir(1)=aPoly(coeffs6,20,t);
            acceldesir(2)=-aPoly(coeffs7,20,t);

            angledesir(2)=-1.0/4.0*M_PI;

            aveldesir(2)=0;

            angleAcceldesir(2)=0;
          }
          if(t>50 && t<57 ) // wait
          {
            posdesir(0)=2.5;
            posdesir(1)=2.5;
            posdesir(2)=2.5;
            vect3_zero(acceldesir);
            vect3_zero(angleAcceldesir);
            vect3_zero(veldesir);
            vect3_zero(aveldesir);
          }
          if(t>57 && t<117 ) // diagonal + rotation 3pi/4
          {
            posdesir(0)=2.5-pPoly(coeffs8x,57,t);
            posdesir(1)=2.5-pPoly(coeffs8y,57,t);
            posdesir(2)=2.5;

            veldesir(0)=-vPoly(coeffs8x,57,t);
            veldesir(1)=-vPoly(coeffs8y,57,t);
            veldesir(2)=0;

            acceldesir(0)=-aPoly(coeffs8x,57,t);
            acceldesir(1)=-aPoly(coeffs8y,57,t);
            acceldesir(2)=0;


            angledesir(2)=-1.0/4.0*M_PI+pPoly(coeffs9,57,t);

            aveldesir(2)=vPoly(coeffs9,57,t);

            angleAcceldesir(2)=aPoly(coeffs9,57,t);
          }
          if(t>117 && t<127 ) // wait
          {
            posdesir(0)=-4;
            posdesir(1)=-3.5;
            posdesir(2)=2.5;
            angledesir(2)=3.0/4.0*M_PI;
            vect3_zero(acceldesir);
            vect3_zero(angleAcceldesir);
            vect3_zero(veldesir);
            vect3_zero(aveldesir);
          }
          if(t>127 && t<152) // go center + rotation pi
          {
            posdesir(0)=-4+pPoly(coeffs10,127,t);
            posdesir(1)=-3.5+pPoly(coeffs10,127,t);
            posdesir(2)=2.0;

            veldesir(0)=vPoly(coeffs10,127,t);
            veldesir(1)=vPoly(coeffs10,127,t);
            veldesir(2)=0;

            acceldesir(0)=aPoly(coeffs10,127,t);
            acceldesir(1)=aPoly(coeffs10,127,t);
            acceldesir(2)=0;


            angledesir(2)=3.0*M_PI/4.0-pPoly(coeffs11,127,t);

            aveldesir(2)=-vPoly(coeffs11,127,t);

            angleAcceldesir(2)=-aPoly(coeffs11,127,t);
          }
          if(t>152 && t<220 ) // wait
          {
            noInt=false;

            posdesir(0)=0;
            posdesir(1)=0;
            posdesir(2)=2.5;
            angledesir(2)=0;
            vect3_zero(acceldesir);
            vect3_zero(angleAcceldesir);
            vect3_zero(veldesir);
            vect3_zero(aveldesir);
          }
          if(t>220)
          {
            pathNb+=1;
            path_debut_time=ros::Time::now().toSec();
          }

        }
        if(pathNb==8)
        {
          t= ros::Time::now().toSec() - path_debut_time;
          if(t<25 ) // go to the beginning
          {
            noInt=true;
            posdesir(0)=0;
            posdesir(1)=2.5;
            posdesir(2)=2.5;
          }
          if(t>25 && t<253 )
          {
            GainCP=0.2;
            maxThrust=40;

            posdesir(0)=r*sin(omega*(t-25));
            posdesir(1)=r*cos(omega*(t-25));
            posdesir(2)=2.5;

            veldesir(0)=omega*r*cos(omega*(t-25));
            veldesir(1)=-omega*r*sin(omega*(t-25));
            veldesir(2)=0;

            acceldesir(0)=-omega*omega*r*sin(omega*(t-25));
            acceldesir(1)=-omega*omega*r*cos(omega*(t-25));
            acceldesir(2)=0;
          }
          if(t>253 && t<263 )
          {
            posdesir(0)=2.5;
            posdesir(1)=0;
            posdesir(2)=2.5;
            vect3_zero(acceldesir);
            vect3_zero(angleAcceldesir);
            vect3_zero(veldesir);
            vect3_zero(aveldesir);
          }
          if(t>263 && t<310)
          {
            posdesir(0)=0;
            posdesir(1)=0;
            posdesir(2)=2.0;
            noInt=false;
          }
          
        }
        if(pathNb==9) // +- 15 yaw
        {
			t=ros::Time::now().toSec();
			posdesir(0)=3.0;
            posdesir(1)=0;
            posdesir(2)=3.0;
            angledesir(0)=0;
            angledesir(1)=0;

            aveldesir(1)=0;
            angleAcceldesir(1)=0;
            

            angledesir(2)=0.35*sin(0.34*(t));

            aveldesir(2)=0.34*0.35*cos(0.34*(t));

            angleAcceldesir(2)=-0.34*0.34*0.35*sin(0.34*(t));
         }
         if(pathNb==10) // +- 15 pitch
        {
			t=ros::Time::now().toSec();
			posdesir(0)=3.0;
            posdesir(1)=0;
            posdesir(2)=3.0;

            angledesir(0)=0;
            angledesir(2)=0;
            
            aveldesir(2)=0;
            angleAcceldesir(2)=0;


            angledesir(1)=0.35*sin(0.34*(t));

            aveldesir(1)=0.34*0.35*cos(0.34*(t));

            angleAcceldesir(1)=-0.34*0.34*0.35*sin(0.34*(t));
         }
         if(pathNb==11) // flip
        {
			t=ros::Time::now().toSec();
			posdesir(0)=3.0;
            posdesir(1)=0;
            posdesir(2)=2.0;

            zero_vel();
            angleAcceldesir(0)=40;
            
            ctrlNb=4;
         }  
        ROS_INFO("Path number %i",pathNb);


    }
    else
    {
      step=0;
      path_debut=true;
      zero_vel();
      maxThrust=100;
      noInt=false;
    }
    
    

    statedesir.header.stamp=ros::Time::now();
    statedesir.pose=vects2pose(posdesir,angledesir);
    statedesir.vel=vects2twist(veldesir,aveldesir);
    statedesir.accel=vects2twist(acceldesir,angleAcceldesir);
    statedesir.maxThrust=maxThrust;
    statedesir.GainCP=GainCP;
    statedesir.noInt=noInt;
    statedesir.ctrlNb=ctrlNb;


    State_pub.publish(statedesir);
    Path_pub.publish(path_info);
    loop_rate.sleep();

  }

  return 0;
}
