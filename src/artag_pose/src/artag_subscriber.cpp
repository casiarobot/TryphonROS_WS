#include <artag_pose/artag_subscriber.h>
#include "ros/ros.h"

ArtagSubscriber::ArtagSubscriber(const std::string& topic_name,
								 const MarkersPose& markers,
								 ros::NodeHandle & nh):
receiveIsFirstMsg(false),
msgReceiveSincePull(false),
topicName(topic_name),
lastReception(ros::Time::now())
{
	ROS_INFO_STREAM("New ArtagSubscriber for " << topic_name);

	timer = nh.createTimer(ros::Duration(5), &ArtagSubscriber::timerCallback, this);

	// Subscribe to topic
	nh.subscribe<ar_track_alvar::AlvarMarkers>(
								topic_name,
								1,
								&ArtagSubscriber::artagCallback,
								this
							);



	lookupCameraTf();
}

void ArtagSubscriber::lookupCameraTf(){

}


void ArtagSubscriber::timerCallback(const ros::TimerEvent& event){
	ros::Duration t = ros::Time::now() - lastReception;
	if(receiveIsFirstMsg && t > ros::Duration(5.0))
		ROS_WARN_STREAM("Topic \"" << topicName
						<< "\" has not respond for " << t.toSec() << "s");
}

void ArtagSubscriber::artagCallback(const ar_track_alvar::AlvarMarkers::ConstPtr& msg){
	if(!receiveIsFirstMsg)
		receiveIsFirstMsg = true;

	ROS_INFO_ONCE("Marker detected by camera");

	lastReception = ros::Time::now();
	msgReceiveSincePull = true;

	std::vector<geometry_msgs::Pose> validPosition;

	// TODO replace by iterator
	for(unsigned i = 0; i < msg->markers.size(); i++){
		if(markers.isValidMarker(msg->markers[i].id))
			validPosition.push_back(msg->markers[i].pose.pose);
		else
			ROS_WARN_STREAM("Topic \"" << sub.getTopic()
							<< "\" invalid tag detected id=" << msg->markers[i].id);
	}
}

bool ArtagSubscriber::wasMsgReceiveSinceLastPull(){
	return msgReceiveSincePull;
}

Eigen::Vector3d ArtagSubscriber::pullAveragePose(){
	msgReceiveSincePull = false;
	return avgPose;
}
