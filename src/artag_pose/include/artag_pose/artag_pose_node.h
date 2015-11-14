#ifndef _ARTAG_POSE_NODE_H_
#define _ARTAG_POSE_NODE_H_

// Standard C/C++ libraries
#include <math.h>
#include <string.h>
#include <vector>
#include <iostream>
#include <ctime>


//library for ros
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <dynamic_reconfigure/server.h>

#include <ar_track_alvar/AlvarMarkers.h>
#include <sensors/compass.h>

#include <artag_pose/ArtagPoseConfig.h> // Auto-generate by a configuration file

// Project includes
#include "artag_pose/artag_subscriber.h"
#include "artag_pose/tf_config_loader.h"
#include "artag_pose/default_config.h"
#include "artag_pose/particle_filter.h"


typedef boost::shared_ptr<ArtagSubscriber> ArtagSubPtr;
class ArtagPoseNode{

	ros::NodeHandle nodeHandle;
	tf::TransformBroadcaster br;

	// Ros parameter:
	int frequency;
	int numMarkers;
	double marker_size;

	DefaultConfig* markerConfig;
	MarkersPosePtr markersPose;

	bool initializing;
	bool settingFirstOffset;
	Eigen::Vector3d offset;

	ros::Time startNode;

	ParticleFilterPtr pf;
	std::vector<ArtagSubPtr> artagSubs;
	ros::Publisher pubPose;
	ros::Publisher pubParticles;
	ros::Publisher pubBestLLParticles;
	ros::Subscriber subCompass;

	dynamic_reconfigure::Server<artag_pose::ArtagPoseConfig> dynamicReconfigServer;
    dynamic_reconfigure::Server<artag_pose::ArtagPoseConfig>::CallbackType dynamicReconfigCallback;

public:
	ArtagPoseNode(ParticleFilter* ppf);
	~ArtagPoseNode();

	void start();


	void compassCallback(const sensors::compass::ConstPtr& msg);
	void dynamicParametersCallback(artag_pose::ArtagPoseConfig &config, uint32_t level);
private:
	void loop();
	void loadConfig();
	std::vector<std::string> loadCameraTopics();
	void createPublishers();
	void createSubscribers();
	void computePoseAndPublish();
	void hardcodeValue2cam(std::list<tagHandle_t> &tagsDetected, unsigned nb1, unsigned nb2);

};

#endif // _ARTAG_POSE_NODE_H_
