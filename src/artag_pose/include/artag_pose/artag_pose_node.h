#ifndef _ARTAG_POSE_NODE_H_
#define _ARTAG_POSE_NODE_H_

// Standard C/C++ libraries
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <ctime>
#include <string.h>
#include <vector>
#include <iostream>


//library for ros
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PolygonStamped.h>

class ArtagPoseNode{
public:
	ArtagPoseNode();
	void start();

	void createPublisherAndSubscriber();
private:
	ros::NodeHandle nodeHandle;

};

#endif // _ARTAG_POSE_NODE_H_
