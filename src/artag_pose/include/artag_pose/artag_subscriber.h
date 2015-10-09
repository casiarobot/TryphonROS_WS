#ifndef ARTAGSUBSCRIBER_H
#define ARTAGSUBSCRIBER_H


#include <string>
#include <vector>

#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_broadcaster.h>

#include <Eigen/Geometry>

#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Pose.h>

#include <ar_track_alvar_msgs/AlvarMarkers.h>

#include "artag_pose/markers_pose.h"




typedef std::vector<ar_track_alvar_msgs::AlvarMarker> TrackedMarker;

typedef struct {
	int idTag;
	Eigen::Affine3d camPose;	/* Relative pose from the camera frame */
	Eigen::Affine3d globalPose; /* Global pose from the world frame */
	double weight;				/* Not normalize weight */
}
tagHandle;

class ArtagSubscriber{

	// For debuging
	tf::TransformBroadcaster br;

	ros::Subscriber sub;

	std::string cameraName, topicName;
	MarkersPosePtr markers;

	bool receiveIsFirstMsg;
	ros::Timer timer;
	ros::Time lastReception;


	Eigen::Affine3d cubeToCam;
	ar_track_alvar_msgs::AlvarMarkers oldMsg;
	unsigned int emptyCount;

	// Configuration
	double jmpThreshold;

	geometry_msgs::Pose avgPose;
	std::list<tagHandle> tagsDetected;
public:
	ArtagSubscriber(const std::string& camera_name,
	                const std::string& topic_name,
	                MarkersPosePtr markers,
	                ros::NodeHandle & nh);

	void artagCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg);
	void timerCallback(const ros::TimerEvent& event);

	void pullTagDetected(std::list<tagHandle>& tagList);

private:
	void lookupCameraTf();
	TrackedMarker::iterator  findMarkerInOldMsgById(unsigned int id);
	double distanceBetweenPoint(geometry_msgs::Point A,
	                            geometry_msgs::Point B);
	Eigen::Affine3d fromRelativePoseToGlobalTf(Eigen::Affine3d& camToTag,
	                                           Eigen::Affine3d worldToTag,
	                                           const int tagName = 0);
	Eigen::Affine3d fromRelativePoseToGlobalTfold(const Eigen::Affine3d& camToTag,
	                                           const Eigen::Affine3d& worldToTag,
	                                           const int tagName = 0);
	tf::Pose getPoseComposition(const tf::Pose& start,
	                            const tf::Pose& increment);

};

#endif // ARTAGSUBSCRIBER_H
