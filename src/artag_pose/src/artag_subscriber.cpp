
#include <tf/transform_listener.h>
#include <artag_pose/artag_subscriber.h>
#include "ros/ros.h"

ArtagSubscriber::ArtagSubscriber(const std::string& camera_name,
								 const std::string& topic_name,
								 MarkersPosePtr markers,
								 ros::NodeHandle & nh):
	cameraName(camera_name),
	topicName(topic_name),
	markers(markers),
	receiveIsFirstMsg(false),
	msgReceiveSincePull(false),
	lastReception(ros::Time::now()),
	emptyCount(0),

	jmpThreshold(1.0)
{
	ROS_INFO_STREAM("New ArtagSubscriber for " << topic_name);



	timer = nh.createTimer(ros::Duration(5), &ArtagSubscriber::timerCallback, this);

	// Subscribe to topic
	sub = nh.subscribe<ar_track_alvar::AlvarMarkers>(
								topic_name,
								1,
								&ArtagSubscriber::artagCallback,
								this
							);



	lookupCameraTf();
}

void ArtagSubscriber::lookupCameraTf(){
	tf::TransformListener listener;
	try{
		listener.waitForTransform("/cube", cameraName, ros::Time(0), ros::Duration(4.0));
		listener.lookupTransform("/cube", cameraName, ros::Time(0), cameraTf);
	}
	catch (tf::TransformException &ex) {
		ROS_ERROR_STREAM("Camera tf not found : " << cameraName);
		ROS_ERROR("%s",ex.what());
		ros::shutdown();
	}
}


void ArtagSubscriber::timerCallback(const ros::TimerEvent& event){
	ros::Duration t = ros::Time::now() - lastReception;
	if(t > ros::Duration(5.0)){
		if(!receiveIsFirstMsg)
			ROS_INFO_STREAM("Topic \"" << topicName
							<< "\" has not respond for " << t.toSec() << "s");
		else
			ROS_WARN_STREAM("Topic \"" << topicName
							<< "\" still has not received is first msg");
	}
}

void ArtagSubscriber::artagCallback(const ar_track_alvar::AlvarMarkers::ConstPtr& msg){
	if(!receiveIsFirstMsg)
		receiveIsFirstMsg = true;

	ROS_INFO_ONCE("Marker detected by camera");

	lastReception = ros::Time::now();
	msgReceiveSincePull = true;

	TrackedMarker trackMarkers = msg->markers;

	if(trackMarkers.empty()){
		ROS_INFO_STREAM("Topic \"" << topicName
						<< "\" no tag detected");
		emptyCount++;
		return;
	}
	else
		emptyCount = 0;

	std::vector<geometry_msgs::Pose> validPosition;
	TrackedMarker::iterator m;
	for(m = trackMarkers.begin(); m != trackMarkers.end(); ++m)
	for(unsigned i = 0; i < trackMarkers.size(); i++){
		// Does the id is in the list of valid markers
		if(markers->isValidMarker(m->id)){
			TrackedMarker::iterator oldM;
			oldM = findMarkerInOldMsgById(m->id);
			// If the tag was not in the old msg
			if(oldM == oldMsg.markers.end()){
				ROS_INFO_STREAM("Topic \"" << topicName
								<< "\" new tag detected id=" << m->id);
			}
			else{
				// When the marker was in the previous Callback
				// the difference between the two is checked
				geometry_msgs::Point newPosition = m->pose.pose.position;
				geometry_msgs::Point oldPosition = oldM->pose.pose.position;
				double dist = distanceBetweenPoint(newPosition, oldPosition);
				if(dist >= jmpThreshold){
					ROS_WARN_STREAM("Tag " << m->id
									<< " move more than the maximum threshold " << dist);
				}
			}

			// The marker is added to pile of valid position
			validPosition.push_back(m->pose.pose);
		}
		else
			ROS_WARN_STREAM("Topic \"" << topicName
							<< "\" invalid tag detected id=" << m->id);
	}

	//ROS_INFO_STREAM("Valid position" << validPosition[0]);



	oldMsg = *msg;
	/*
	 * Add emptyCounter for setting the oldMsg when too much time has pass?
	*		2) Check for invalid tag, print WARN in consequence; Samething for no tag
	*		3) If no timeout, compare the last tag to see if we look at new tags
	*		4a) If old tag are found, check if the position did move more from certain threshold and warn
	*		4b) If new tag, we could add warn or maybe a security to prevent a jump?
	*		5) Do the reverse transformation from the camera relative to global position
	*		6) Get the average pose and print the average deviation (maybe add warning when too large)
	*/
}

TrackedMarker::iterator ArtagSubscriber::findMarkerInOldMsgById(unsigned int id){
	std::vector<ar_track_alvar::AlvarMarker>::iterator it;
	for(it = oldMsg.markers.begin(); it != oldMsg.markers.end(); ++it){
		if(id == it->id)
			return it;
	}
	return it;

}

double ArtagSubscriber::distanceBetweenPoint(geometry_msgs::Point A,
											 geometry_msgs::Point B){
	double mag = (A.x - B.x)*(A.x - B.x) +
				 (A.y - B.y)*(A.y - B.y) +
				 (A.z - B.z)*(A.z - B.z);

	return fabs(mag);
}

bool ArtagSubscriber::wasMsgReceiveSinceLastPull(){
	return msgReceiveSincePull;
}

Eigen::Vector3d ArtagSubscriber::pullAveragePose(){
	msgReceiveSincePull = false;
	return avgPose;
}
