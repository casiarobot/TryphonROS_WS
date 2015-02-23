#include "ros/ros.h"
#include "sensors/sonar.h"
#include "sensors/sonarArray.h"
#include "sensors/compass.h"
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include "sensors/imuros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"

#include <fstream>
using namespace std;

std::ofstream myfile;
std::ofstream myfile1;
std::ofstream myfile2;
std::ofstream myfile3;
double debut=0;
bool start= true;

double ax=0;
double ay=0;
double az=0;
double gx=0;
double gy=0;
double gz=0;
double mx=0;
double my=0;
double mz=0;

void subSonar(const sensors::sonarArray::ConstPtr& msg)
{
  if(start && msg->sonars.size()>0){
     myfile << "D,";
     for (int i=0; i<msg->sonars.size()-1; ++i)
         {const sensors::sonar &sonar = msg->sonars[i];
          myfile << sonar.id <<",";}
     const sensors::sonar &sonar = msg->sonars[msg->sonars.size()-1];
     myfile << sonar.id <<endl;
     start=false;
  }
  for (int j=0;j<10;j++)
  {
  myfile << "D" << j << ","; //print the D0, and the comma of each in first column
  for (int i=0; i<msg->sonars.size(); ++i)
    {

    const sensors::sonar &sonar = msg->sonars[i];
    // ROS_INFO_STREAM("ID: " << sonar.id << " - D0: " << sonar.distance[0] << ", D1: " << sonar.distance[1]);
    myfile << sonar.distance[j];  //prints a row of  the sonar distance of D[j], each colum is seperate sonar
    if (i!=msg->sonars.size()-1) {myfile <<",";} // add a , except for last row

    }
  double secs = ros::Time::now().toSec();
  myfile << ","<< secs-debut <<endl; //end the line
  }

}

void subComp(const sensors::compass::ConstPtr& msg)
{
      //ROS_INFO_STREAM("ID: " << msg->id << " - RZ0: " << msg->rz[0] << ", RZ1: " << msg->rz[1]);
      double secs = ros::Time::now().toSec();
      myfile1 <<msg->rz[0] <<","<< secs-debut <<endl; //end the line
}


void subForces(const geometry_msgs::WrenchStamped ForceS)
{
      double secs = ros::Time::now().toSec();
      geometry_msgs::Wrench Force=ForceS.wrench;
      myfile1 <<Force.force.x <<","<<Force.force.y <<","<<Force.force.z <<","<<Force.torque.x <<","<<Force.torque.y <<","<<Force.torque.z <<","<< secs-debut <<endl; //end the line
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
      double secs = ros::Time::now().toSec();
      myfile3 << ax << ","<< ay <<","<< az <<","<< gx <<","<< gy <<","<< gz <<","<< mx <<","<< my <<","<< mz<<"," <<secs-debut <<endl;

      ROS_INFO("ax: %f, ay: %f, az: %f",ax,ay,az);
      //ROS_INFO("gx: %f, gy: %f, gz: %f",gx,gy,gz);
      //ROS_INFO("mx: %f, my: %f, mz: %f",mx,my,mz);
}

/*void subMCPTAM(const geometry_msgs::PoseArray Aposes) // with MCPTAM
{
    geometry_msgs::Pose pose=Aposes.poses[0];*/
void subMCPTAM(const geometry_msgs::PoseStamped Aposes)   // with Gazebo
{
    geometry_msgs::Pose pose=Aposes.pose;
    double secs = ros::Time::now().toSec();
    myfile << pose.position.x << ","<< pose.position.y <<","<< pose.position.z <<","<< pose.orientation.x <<","<< pose.orientation.y <<","<< pose.orientation.z << "," << pose.orientation.w << "," <<secs-debut <<endl;


}

int main(int argc, char **argv)
{
/*char link[100]="/home/py/Dropbox/Tryphon_PPY/Experiments_August2014/Matlab/csvfiles/Thrust50/Sonars/";// make sure it is the right path
char link1[100]="/home/py/Dropbox/Tryphon_PPY/Experiments_August2014/Matlab/csvfiles/Thrust50/Compass/";
char link2[100]="/home/py/Dropbox/Tryphon_PPY/Experiments_August2014/Matlab/csvfiles/Thrust50/Force/";
char link3[100]="/home/py/Dropbox/Tryphon_PPY/Experiments_August2014/Matlab/csvfiles/Thrust50/IMU/";
strcat(link,argv[1]);
strcat(link1,argv[1]);
strcat(link2,argv[1]);
strcat(link3,argv[1]);
strcat(link,"_sonars.csv");
strcat(link1,"_compass.csv");
strcat(link2,"_forces.csv");
strcat(link3,"_imu.csv");

myfile.open(link);// make sure it is the right path
myfile1.open(link1);// make sure it is the right pathArray
myfile2.open(link2);// make sure it is the right path
myfile3.open(link3);// make sure it is the right path
ROS_INFO(link);
ROS_INFO(link1);
ROS_INFO(link2);
ROS_INFO(link3);*/

char link[100]="//home/tryphon/Gazebo/Tests/";
strcat(link,argv[1]);
strcat(link,"_pose.csv");
myfile.open(link);
ROS_INFO(link);

char link1[100]="//home/tryphon/Gazebo/Tests/";
strcat(link1,argv[1]);
strcat(link1,"_thrust.csv");
myfile1.open(link1);
ROS_INFO(link1);


if (myfile && myfile1 )//&& myfile2)
  {
  /*myfile << "Run"<< endl;
  myfile1 << "Run"<< endl << "Z_Angle,time" << endl;
  myfile2 << "Run"<< endl << "Forces,time" << endl  ;
  myfile3 << "Run"<< endl << "ax,ay,az,gx,gy,gz,mx,my,mz,time" << endl  ;*/


  myfile << "Run"<< endl << "x,y,z,qx,qy,qz,qw,time" << endl  ;
  myfile1 << "Run"<< endl << "Fx,Fy,Fz,Tx,Ty,Tz,time" << endl  ;
  ros::init(argc, argv, "csvmaker");
  ros::NodeHandle n;
  debut = ros::Time::now().toSec();
  ros::Subscriber subS = n.subscribe("sonars",1, subSonar);
  ros::Subscriber subC = n.subscribe("compass",1,subComp);
  ros::Subscriber subF = n.subscribe("tryphon/thrust",1,subForces);
  ros::Subscriber subI = n.subscribe("imu",1,subImu);
  ros::Subscriber subM = n.subscribe("mcptam/tracker_pose_array",1,subMCPTAM);

  ros::Rate r(10); // 10 hz
  while (ros::ok())
    {
      ros::spinOnce();
      r.sleep();
    }

  }

  else
  {
    cout <<"Can't open csv file."<<endl;
  }

  myfile.close();
  myfile1.close();
  /*myfile2.close();
  myfile3.close();*/
}

