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
#include "artag_pose/particle_filter.h"


typedef boost::shared_ptr<ParticleFilter> ParticleFilterPtr;
typedef std::vector<ar_track_alvar_msgs::AlvarMarker> TrackedMarker;


class ArtagSubscriber{

	ParticleFilterPtr pf;

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
	std::list<tagHandle_t> tagsDetected;
public:
	ArtagSubscriber(const std::string& camera_name,
	                const std::string& topic_name,
	                MarkersPosePtr markers,
	                ros::NodeHandle& nh,
	                ParticleFilterPtr pf);

	void artagCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg);
	void timerCallback(const ros::TimerEvent& event);

	void pullTagDetected(std::list<tagHandle_t>& tagList);

private:
	void lookupCameraTf();
	TrackedMarker::iterator  findMarkerInOldMsgById(unsigned int id);
	double distanceBetweenPoint(geometry_msgs::Point A,
	                            geometry_msgs::Point B);


};

#endif // ARTAGSUBSCRIBER_H
