
#include <tf/transform_listener.h>
#include "artag_pose/tf_config_loader.h"

TfConfigLoader::TfConfigLoader(){
}


void TfConfigLoader::load(int numMarkers){
	tf::TransformListener listener;
	tf::StampedTransform stampedTf;
	geometry_msgs::TransformStamped stampedTfMsg;
	char tfName[100];

	// The first tf is always unreachable on the first try
	try{
		listener.waitForTransform("/cafeteria", "/cafeteria", ros::Time(0), ros::Duration(1.0));
		listener.lookupTransform("/cafeteria", "/cafeteria", ros::Time(0), stampedTf);
	}
	catch (tf::TransformException &ex) {}

	// The id start at 1
	for(int i = 1; i <= numMarkers; i++){
		sprintf(tfName, "/artag_%02d_link", i);
		try{
			listener.waitForTransform("/cafeteria", tfName, ros::Time(0), ros::Duration(1.0));
			listener.lookupTransform("/cafeteria", tfName, ros::Time(0), stampedTf);
		}
		// If we can't find the tf that means that marker is not used
		catch (tf::TransformException &ex) {
			ROS_ERROR_STREAM("Tf not found : " << tfName);
			ROS_ERROR("%s",ex.what());
			continue;
		}
		tf::transformStampedTFToMsg(stampedTf, stampedTfMsg);
		m.insert(i, stampedTfMsg.transform);
		ROS_INFO_STREAM("Tf retrieve for " << tfName);
	}

}

MarkersPose TfConfigLoader::parse(){
	return m;
}
