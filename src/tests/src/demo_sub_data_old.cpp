#include "ros/ros.h"
#include "sensors/sonar.h"
#include "sensors/sonarArray.h"
#include "sensors/compass.h"

#include <fstream>
using namespace std;
 
std::ofstream myfile; 

void subSonar(const sensors::sonarArray::ConstPtr& msg)
{
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
  myfile <<endl; //end the line
  }   

}
 
void subComp(const sensors::compass::ConstPtr& msg)
{
    ROS_INFO_STREAM("ID: " << msg->id << " - RZ0: " << msg->rz[0] <<
                    ", RZ1: " << msg->rz[1]);
}
 
int main(int argc, char **argv)
{
char link[100]="/opt/ros/groovy/catkin_ws/src/";// make sure it is the right path
strcat(link,argv[1]);


myfile.open(link);// make sure it is the right path

if (myfile)
  {
  myfile << "Run"<< endl;
  myfile << "ID, s1,s2,s3,s4, etc"<<endl;

  ros::init(argc, argv, "sensorsSubscriber");
  ros::NodeHandle n;    
  ros::Subscriber subS = n.subscribe("sonars", 1, subSonar);
  ros::Subscriber subC = n.subscribe("compass",1,subComp);
  ros::spin();
  }
 
  else
  {
    cout <<"Can't open csv file."<<endl;
  }

  myfile.close();
}

