#ifndef ARTAGSUBSCRIBER_H
#define ARTAGSUBSCRIBER_H


#include <string>
#include <vector>

#include "ros/ros.h"

#include <Eigen/Geometry>

#include <geometry_msgs/Pose.h>
#include <ar_track_alvar/AlvarMarkers.h>

#include "artag_pose/markers_pose.h"

class ArtagSubscriber{

ros::Subscriber sub;
MarkersPose markers;

bool receiveIsFirstMsg;
ros::Timer timer;
ros::Time lastReception;

Eigen::Vector3d avgPose;
bool msgReceiveSincePull;

public:
	ArtagSubscriber(const std::string& topic_name,
					const MarkersPose& markers,
					ros::NodeHandle & nh);

	void artagCallback(const ar_track_alvar::AlvarMarkers::ConstPtr& msg);
	void timerCallback(const ros::TimerEvent& event);

	bool wasMsgReceiveSinceLastPull();
	Eigen::Vector3d pullAveragePose();
private:
	void lookupCameraTf();
};

#endif // ARTAGSUBSCRIBER_H
