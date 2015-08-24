#include "artag_pose/artag_pose_node.h"



ArtagPoseNode::ArtagPoseNode():nodeHandle("~"){
	nodeHandle.param<bool>("useYAML", useYAML, false);

	nodeHandle.param<int>("hz", frequency, 10);

	nodeHandle.param<int>("numMarkers", numMarkers, 4);

	if(useYAML){
		ROS_INFO("Using YAML for markers pose");
		markerConfig = new YamlConfigLoader(&nodeHandle);
	}
	else{
		ROS_INFO("Using TF for markers pose");
		markerConfig = new TfConfigLoader;
	}
}
ArtagPoseNode::~ArtagPoseNode(){
	artagSubs.clear();
	delete markerConfig;
}


void ArtagPoseNode::start(){
	loadConfig();
	createSubscribers();
	loop();
}
void ArtagPoseNode::loadConfig(){
	markerConfig->load(numMarkers);
	markersPose = markerConfig->parse();


}

void ArtagPoseNode::createSubscribers(){
	// Get topic list from parameter
	//std::vector<std::string> camera_topics = loadCameraTopics();

	// Bypass the Yaml configuration for debugging
	std::vector<std::string> camera_topics;
	camera_topics.push_back("camera1");
	camera_topics.push_back("/192_168_10_243/artags/artag1/ar_pose_marker");

	// for each camera topics create a subscriber
	std::vector<std::string>::iterator topic_name;
	for(topic_name = camera_topics.begin(); topic_name != camera_topics.end(); topic_name++){
		std::string camera_name = *topic_name;
		// The following element is the topic name
		topic_name++;
		ArtagSubPtr ptr(new ArtagSubscriber(camera_name, *topic_name, markersPose, nodeHandle));
		artagSubs.push_back(ptr);
	}
}


std::vector<std::string> ArtagPoseNode::loadCameraTopics(){
	std::vector<std::string> camera_topics;

	bool parameter_provided = nodeHandle.getParam("camera_topics", camera_topics);

	if(!parameter_provided){
		ROS_ERROR("Can't load list of camera's topic from parameter \"camera_topics\"");
		ros::shutdown();
	}

	return camera_topics;
}

void ArtagPoseNode::loop(){
	ros::Rate loop_rate(frequency);
	while(ros::ok()){

		computePoseAndPublish();

		ros::spinOnce();
		loop_rate.sleep();
	}
}

void ArtagPoseNode::computePoseAndPublish(){

}



int main(int argc, char **argv){

	/*
	 *			==========		General draft of how the node works		=========
	 *										  ***
	 * A) Take the position of an array of artag from an origin from a YAML File or a TF
	 *		1) ConfigLoader virtual class with two implementation YAMLConfigLoader and TFConfigLoader
	 *		2) Load and Parse
	 *			-> Parse return a object that will be pass to the subscriber
	 *				-> struct{ artag_id, pose: x, y, z}
	 *				-> Or map<int, geometry_msgs::pose>
	 *				-> Or the clean code interface of a map< ... > with the name MarkersPose
	 *					->
	 * B) After the configuration is done, it is send to another module which is the subscriber
	 *	  to the artag_track node output
	 * C) Since we might have multiple camera the best option is to have one instance of
	 *	  the subscriber per camera
	 *		1) Either we go throught all the topic to find the one publishing artag
	 *		2) Or we hardcode a list of those topics into a parameter field like this example:
	 *			http://answers.ros.org/question/12321/how-to-write-and-parse-yaml-file-for-ros/
	 *		3) Add it to the ConfigLoader
	 * D) The subscriber:
	 *		1) First off have a timeout to print a warning in the case of artag_track crash
	 *		2) Check for invalid tag, print WARN in consequence; Samething for no tag
	 *		3) If no timeout, compare the last tag to see if we look at new tags
	 *		4a) If old tag are found, check if the position did move more from certain threshold and warn
	 *		4b) If new tag, we could add warn or maybe a security to prevent a jump?
	 *		5) Do the reverse transformation from the camera relative to global position
	 *		6) Get the average pose and print the average deviation (maybe add warning when too large)
	 * E) Publisher: Pose Publisher that handle the multiple state that the subcriber can be in.
	 *		1) Timeout on all sub
	 *			-> No publishing of a pose WARN
	 *		2) Valid pose on all sub
	 *			-> Average of their average
	 *			-> or Average of their individual tag pose
	 *			-> Check for deviation or too big of jump t-1
	 *				-> ignore for t0 or timeout
	 *			-> Publish
	 *		3) Valid pose on some sub
	 *			-> Same than 2), except add some smoothing aka average from the previous pose to prevent jump
	 * F) Maybe use the confidence parameter from the ar_track_alvar message
	 *
	 */

	ros::init(argc, argv, "artag_pose_node");

	ArtagPoseNode mw;

	mw.start();

	return 0;
}
