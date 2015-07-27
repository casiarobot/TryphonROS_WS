#include "artag_pose/artag_pose_node.h"


ArtagPoseNode::ArtagPoseNode(){

}

void ArtagPoseNode::start(){

}

void ArtagPoseNode::createPublisherAndSubscriber(){}



int main(int argc, char **argv){

	ros::init(argc, argv, "artag_pose_node");

	ArtagPoseNode mw;

	mw.start();

	return 0;
}
