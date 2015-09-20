#include "artag_pose/test_pose_node.h"


TestPoseNode::TestPoseNode(){

}

void TestPoseNode::start(){

}

void TestPoseNode::createPublisherAndSubscriber(){

}



int main(int argc, char **argv){
	/*
	 *			==========		General draft of how the node works		=========
	 *
	 * A) Two subscriber callback, one for ground true from gazebo the other from
	 *	  the pose estimator
	 * B) Gound true is save in a attribute
	 * C) In pose estimator callback:
	 *	  -> Publish path with base frame_id is cafeteria for pose estimator and ground true
	 *	  -> Publish error over time in x, y and z using a point msg
	 * D) In a the main loop control the trajectory of the cube, either:
	 *	  ->  "teleport" the cube at various point in the map(how?)
	 *	  ->  Use a control for doing realistic trajectory
	 *		-> I want a do a circle trajectory
	 *		-> do a 360
	 *		-> do both
	 *	  -> One that doesn't move
	 * E) A dynamic reconfiguration change the state of the trajectory
	 *	  -> For example, a certain limit to where the cube teleport (boxing box)
	 *	  -> Threshold on error (change color of the path on point to far, or use distance has
	 *		 a color gradient)
	 *    -> Pause/play button
	 *
	 */


    ros::init(argc, argv, "artag_pose_node");

    TestPoseNode mw;

    mw.start();

    return 0;
}
