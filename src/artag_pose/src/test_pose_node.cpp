#include "artag_pose/test_pose_node.h"


TestPoseNode::TestPoseNode(){

}

void TestPoseNode::start(){

}

void TestPoseNode::createPublisherAndSubscriber(){

}



int main(int argc, char **argv){
    ros::init(argc, argv, "artag_pose_node");

    TestPoseNode mw;

    mw.start();

    return 0;
}
