#include "ros/ros.h"
#include "sensors/sonar.h"
#include "sensors/sonarArray.h"
#include "sensors/compass.h"
#include <geometry_msgs/Wrench.h>
#include "sensors/imuros.h"

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
 

void subForces(const geometry_msgs::Wrench Force)
{
      double secs = ros::Time::now().toSec();
      myfile2 <<Force.force.x <<","<< secs-debut <<endl; //end the line
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

int main(int argc, char **argv)
{
char link[100]="/home/py/Dropbox/Tryphon_PPY/Experiments_August2014/Matlab/csvfiles/Thrust50/Sonars/";// make sure it is the right path
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
myfile1.open(link1);// make sure it is the right path
myfile2.open(link2);// make sure it is the right path
myfile3.open(link3);// make sure it is the right path
ROS_INFO(link);
ROS_INFO(link1);
ROS_INFO(link2);
ROS_INFO(link3);

if (myfile3)// && myfile1 && myfile2)
  {
  myfile << "Run"<< endl;
  myfile1 << "Run"<< endl << "Z_Angle,time" << endl;
  myfile2 << "Run"<< endl << "Forces,time" << endl  ;
  myfile3 << "Run"<< endl << "ax,ay,az,gx,gy,gz,mx,my,mz,time" << endl  ;

  ros::init(argc, argv, "sensorsSubscriber");
  ros::NodeHandle n;    
  debut = ros::Time::now().toSec();
  ros::Subscriber subS = n.subscribe("sonars",1, subSonar);
  ros::Subscriber subC = n.subscribe("compass",1,subComp);
  ros::Subscriber subF = n.subscribe("final_forces",1,subForces);
  ros::Subscriber subI = n.subscribe("imu",1,subImu);
  ros::spin();
  }
 
  else
  {
    cout <<"Can't open csv file."<<endl;
  }

  myfile.close();
  myfile1.close();
  myfile2.close();
  myfile3.close();
}

