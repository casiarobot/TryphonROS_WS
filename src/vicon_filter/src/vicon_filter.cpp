
// %Tag(FULLTEXT)%
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <boost/circular_buffer.hpp>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <fcntl.h>
#include <fstream>
#include <queue>

#ifdef _WIN32
   #include <windows.h>
   #include <time.h>
#else 
#ifdef __QNX__
   #include <unix.h>
   #include <errno.h>
   #include <sys/ioctl.h>
   #include <unistd.h>
#endif
   #include <netinet/in.h>
   #include <netdb.h>
   #include <sys/socket.h>
   #include <sys/time.h>
   #include <sys/select.h>
#endif /* _WIN32 */


//ros::Time beginTime;

//#include "vicon_filter.h"

// INITIALIZE STRUCTS
// typedef struct discreteTF discreteTF;

// struct discreteTF {  //second order discrete TF
//   double a0;
//   double a1;
//   double a2;
//   double b0;
//   double b1;
//   double b2;
// } posnTF, velocityTF;

double posnTF_a0 = 0.3679;
double posnTF_a1 = -1.2811;
double posnTF_b0 = 0.0363;
double posnTF_b1 = 0.0505;

double velocityTF_a0 = 0.3679;
double velocityTF_a1 = -1.2811;
double velocityTF_b0 = -5.1485;
double velocityTF_b1 = 5.1485; 

float Ts = 0.0166666;

// INITIALIZE VARIABLES
geometry_msgs::PoseStamped estimatedPoseMsg; //message published to vicon_filter topic
geometry_msgs::TwistStamped estimatedTwistMsg; //message published to vicon_filter topic
bool isFirstCallback = true;
bool isSecondCallback = false;
double xPosnMeasured;
double yPosnMeasured;
double zPosnMeasured;

//memory variables for filtering
geometry_msgs::Pose estimatedPosePrev;
geometry_msgs::Pose estimatedPosePrevPrev;

geometry_msgs::Pose measuredPosePrev;
geometry_msgs::Pose measuredPosePrevPrev;

geometry_msgs::Twist estimatedTwistPrev;
geometry_msgs::Twist estimatedTwistPrevPrev;

// geometry_msgs::Pose initPose(){
//   geometry_msgs::Pose returnPose;
//   returnPose.position.x = 0.0;
//   returnPose.position.
//   return returnPose;
// }

ros::Time difRosTime(ros::Time t2, ros::Time t1)
{
    ros::Duration timeDiff = t2 - t1;
    return t2 - timeDiff;
    //return t2.toSec()-t1.toSec();
}

void viconPoseCallback(geometry_msgs::PoseStamped measuredPoseStamped){
  
  if(isFirstCallback){
    estimatedPoseMsg.pose = measuredPoseStamped.pose;  
    estimatedTwistMsg.twist.linear.x = 0.0;
    estimatedTwistMsg.twist.linear.y = 0.0;
    estimatedTwistMsg.twist.linear.z = 0.0;

    estimatedPosePrevPrev = estimatedPoseMsg.pose;
    measuredPosePrevPrev = measuredPoseStamped.pose;
    estimatedTwistPrevPrev = estimatedTwistMsg.twist;

    isFirstCallback = false;
    isSecondCallback = true;
  }
  else if(isSecondCallback){
    estimatedPoseMsg.pose = measuredPoseStamped.pose; 

    estimatedTwistMsg.twist.linear.x = (estimatedPoseMsg.pose.position.x - estimatedPosePrevPrev.position.x)/Ts;
    estimatedTwistMsg.twist.linear.y = (estimatedPoseMsg.pose.position.y - estimatedPosePrevPrev.position.y)/Ts;
    estimatedTwistMsg.twist.linear.z = (estimatedPoseMsg.pose.position.z - estimatedPosePrevPrev.position.z)/Ts;

    estimatedPosePrev = estimatedPoseMsg.pose;
    measuredPosePrev = measuredPoseStamped.pose;
    estimatedTwistPrev = estimatedTwistMsg.twist;

    isSecondCallback = false;
  }
  else{
  
    //ROS_INFO("Position Received: x = %f",measuredPoseStamped.pose.position.x); 
  
    //filter position
    xPosnMeasured = measuredPoseStamped.pose.position.x;
    yPosnMeasured = measuredPoseStamped.pose.position.y;
    zPosnMeasured = measuredPoseStamped.pose.position.z;

    estimatedPoseMsg.pose.position.x = posnTF_b1*measuredPosePrev.position.x + posnTF_b0*measuredPosePrevPrev.position.x
                                       -posnTF_a1*estimatedPosePrev.position.x - posnTF_a0*estimatedPosePrevPrev.position.x;
    estimatedPoseMsg.pose.position.y = posnTF_b1*measuredPosePrev.position.y + posnTF_b0*measuredPosePrevPrev.position.y
                                       -posnTF_a1*estimatedPosePrev.position.y - posnTF_a0*estimatedPosePrevPrev.position.y;
    estimatedPoseMsg.pose.position.z = posnTF_b1*measuredPosePrev.position.z + posnTF_b0*measuredPosePrevPrev.position.z
                                       -posnTF_a1*estimatedPosePrev.position.z - posnTF_a0*estimatedPosePrevPrev.position.z;
                                   
    estimatedTwistMsg.twist.linear.x = velocityTF_b1*measuredPosePrev.position.x + velocityTF_b0*measuredPosePrevPrev.position.x
                    -velocityTF_a1*estimatedTwistPrev.linear.x - velocityTF_a0*estimatedTwistPrevPrev.linear.x;
    estimatedTwistMsg.twist.linear.y = velocityTF_b1*measuredPosePrev.position.y + velocityTF_b0*measuredPosePrevPrev.position.y
                    -velocityTF_a1*estimatedTwistPrev.linear.y - velocityTF_a0*estimatedTwistPrevPrev.linear.y;
    estimatedTwistMsg.twist.linear.z = velocityTF_b1*measuredPosePrev.position.z + velocityTF_b0*measuredPosePrevPrev.position.z
                    -velocityTF_a1*estimatedTwistPrev.linear.z - velocityTF_a0*estimatedTwistPrevPrev.linear.z;

   
    //update memory variables
    estimatedPosePrevPrev = estimatedPosePrev;
    estimatedPosePrev = estimatedPoseMsg.pose;
    estimatedTwistPrevPrev = estimatedTwistPrev;
    estimatedTwistPrev = estimatedTwistMsg.twist;

    measuredPosePrevPrev = measuredPosePrev;
    measuredPosePrev = measuredPoseStamped.pose;
  }
  estimatedPoseMsg.header.stamp = ros::Time::now();
  estimatedTwistMsg.header.stamp = ros::Time::now();
  
}
// %EndTag(CALLBACK)%

int main(int argc, char **argv){

  ros::init(argc, argv, "vicon_filter");

  ros::NodeHandle node;

  ros::Publisher filteredPose = node.advertise<geometry_msgs::PoseStamped>("vicon_filter/pose",1000);
  ros::Publisher filteredTwist = node.advertise<geometry_msgs::TwistStamped>("vicon_filter/twist",1000);

  // ros::Subscriber sub = node.subscribe("vicon_pose", 1000, viconPoseCallback);
  ros::Subscriber sub = node.subscribe("vicon_sim", 1000, viconPoseCallback);


  ros::Rate loopRate(60.0);

  //init variables
  //beginTime = ros::Time::now();

  while (node.ok()){
    ros::spinOnce();
    //ros::Time nowTime = ros::Time::now();

// %Tag(PUBLISH)%
    filteredPose.publish(estimatedPoseMsg);
    filteredTwist.publish(estimatedTwistMsg);
// %EndTag(PUBLISH)%
 
  //publish filtered pose
   

// %Tag(RATE_SLEEP)%
    loopRate.sleep();
// %EndTag(RATE_SLEEP)%
  } 


  return 0;
}

