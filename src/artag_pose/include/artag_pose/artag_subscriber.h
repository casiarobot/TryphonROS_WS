#ifndef ARTAGSUBSCRIBER_H
#define ARTAGSUBSCRIBER_H


#include <string>
#include <vector>

#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <Eigen/Geometry>

#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Pose.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>

#include "artag_pose/markers_pose.h"

typedef std::vector<ar_track_alvar_msgs::AlvarMarker> TrackedMarker;

class ArtagSubscriber{

ros::Subscriber sub;

std::string cameraName, topicName;
MarkersPosePtr markers;

bool receiveIsFirstMsg;
ros::Timer timer;
ros::Time lastReception;


tf::StampedTransform cubeToCamTf;
ar_track_alvar_msgs::AlvarMarkers oldMsg;
unsigned int emptyCount;

// Configuration
double jmpThreshold;

Eigen::Vector3d avgPose;
bool msgReceiveSincePull;

public:
	ArtagSubscriber(const std::string& camera_name,
					const std::string& topic_name,
					MarkersPosePtr markers,
					ros::NodeHandle & nh);

	void artagCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg);
	void timerCallback(const ros::TimerEvent& event);

	bool wasMsgReceiveSinceLastPull();
	Eigen::Vector3d pullAveragePose();

private:
	void lookupCameraTf();
	TrackedMarker::iterator  findMarkerInOldMsgById(unsigned int id);
	double distanceBetweenPoint(geometry_msgs::Point A,
								geometry_msgs::Point B);
	geometry_msgs::Pose fromRelativePoseToGlobalTf(const tf::Pose& camToTag,
										const tf::Pose& worldToTag);
	tf::Pose getPoseComposition(const tf::Pose& start,
				const tf::Pose& increment);

};

#endif // ARTAGSUBSCRIBER_H
