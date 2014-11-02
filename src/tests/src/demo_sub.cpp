#include "ros/ros.h"
#include "sensors/sonar.h"
#include "sensors/sonarArray.h"
#include "sensors/compass.h"

void subSonar(const sensors::sonarArray::ConstPtr& msg)
{
  for (int i=0; i<msg->sonars.size(); ++i)
  {
   const sensors::sonar &sonar = msg->sonars[i];
    ROS_INFO_STREAM("ID: " << sonar.id << " - D0: " << sonar.distance[0] <<
                    ", D1: " << sonar.distance[1]);
  }
}

void subComp(const sensors::compass::ConstPtr& msg)
{
    ROS_INFO_STREAM("ID: " << msg->id << " - RZ0: " << msg->rz[0] <<
                    ", RZ1: " << msg->rz[1]);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sensorsSubscriber");
  ros::NodeHandle n;	
  ros::Subscriber subS = n.subscribe("sonars", 1000, subSonar);
  ros::Subscriber subC = n.subscribe("compass",1000,subComp);
  ros::spin();
}
