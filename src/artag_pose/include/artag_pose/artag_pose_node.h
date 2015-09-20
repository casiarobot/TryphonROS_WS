#ifndef _ARTAG_POSE_NODE_H_
#define _ARTAG_POSE_NODE_H_

// Standard C/C++ libraries
#include <math.h>
#include <string.h>
#include <vector>
#include <iostream>


//library for ros
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PolygonStamped.h>

#include <ar_track_alvar_msgs/AlvarMarkers.h>

// Project includes
#include "artag_pose/artag_subscriber.h"
#include "artag_pose/yaml_config_loader.h"
#include "artag_pose/tf_config_loader.h"


typedef boost::shared_ptr<ArtagSubscriber> ArtagSubPtr;
class ArtagPoseNode{

	ros::NodeHandle nodeHandle;

	// Ros parameter:
	bool useYAML;
	int frequency;
	int numMarkers;

	ConfigLoader* markerConfig;
	MarkersPosePtr markersPose;

	std::vector<ArtagSubPtr> artagSubs;
public:
	ArtagPoseNode();
	~ArtagPoseNode();

	void start();
private:
	void loop();
	void loadConfig();
	std::vector<std::string> loadCameraTopics();
	void createSubscribers();
	void computePoseAndPublish();

};

#endif // _ARTAG_POSE_NODE_H_
