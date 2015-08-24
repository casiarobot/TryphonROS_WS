
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
	/*try{
		listener.waitForTransform("/cafeteria", "/cafeteria", ros::Time(0), ros::Duration(1.0));
		listener.lookupTransform("/cafeteria", "/cafeteria", ros::Time(0), stampedTf);
	}
	catch (tf::TransformException &ex) {}(*/
	int tries = 0;
	for(int i = 0; i < numMarkers; i++){
		sprintf(tfName, "/artag_%02d_background_link", i);
		try{
			listener.waitForTransform("/cafeteria", tfName, ros::Time(0), ros::Duration(10.0));
			listener.lookupTransform("/cafeteria", tfName, ros::Time(0), stampedTf);
		}
		// If we can't find the tf that means that marker is not used
		catch (tf::TransformException &ex) {
			ROS_ERROR_STREAM("Tf not found : " << tfName);
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();

			// After 3 attemp give up
			if(tries < 3){
				tries++;
				i--;
			}
			else
				tries = 0;
			continue;
		}

		// Add tag to the tag map
		m->insert(i, stampedTf);
		ROS_INFO_STREAM("Tf retrieve for " << tfName);
	}

}

MarkersPosePtr TfConfigLoader::parse(){
	return m;
}
