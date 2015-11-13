#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <artag_pose/artag_subscriber.h>
#include "ros/ros.h"

ArtagSubscriber::ArtagSubscriber(const std::string& camera_name,
                                 const std::string& topic_name,
								 const Eigen::Matrix4d cube_cam,
								 MarkersPosePtr markers,
								 ros::NodeHandle & nh,
                                 ParticleFilterPtr pf):
	cameraName(camera_name),
    cube2Cam(cube_cam),
	topicName(topic_name),
	markers(markers),
	receiveIsFirstMsg(false),
	lastReception(ros::Time::now()),
	emptyCount(0),
	jmpThreshold(1.0)
{
	ROS_INFO_STREAM("New ArtagSubscriber for " << topic_name);



	timer = nh.createTimer(ros::Duration(5), &ArtagSubscriber::timerCallback, this);

	// Subscribe to topic
	sub = nh.subscribe<ar_track_alvar_msgs::AlvarMarkers>(
								topic_name,
								10,
								&ArtagSubscriber::artagCallback,
								this
							);



	//lookupCameraTf();
}

void ArtagSubscriber::lookupCameraTf(){
	/*
	tf::TransformListener listener;
	tf::StampedTransform cubeToCamTf;
	try{
		listener.waitForTransform("/cube", cameraName, ros::Time(0), ros::Duration(10.0));
		listener.lookupTransform("/cube", cameraName, ros::Time(0), cubeToCamTf);
		//tf::poseTFToEigen(cubeToCamTf, cubeToCam);

		// TODO: Harcoded
		Eigen::Matrix3d cube2Cam_R_mat;
		/*cube2Cam_R_mat << 0, -1,  0,
					      0,  0, -1,
					      1,  0,  0;
		cubeToCam.linear() = cube2Cam_R_mat;

	}
	catch (tf::TransformException &ex) {
		ROS_ERROR_STREAM("Camera tf not found : " << cameraName);
		ROS_ERROR("%s",ex.what());
		ros::shutdown();
	}*/
}


void ArtagSubscriber::timerCallback(const ros::TimerEvent& event){
	ros::Duration t = ros::Time::now() - lastReception;
	if(t > ros::Duration(5.0)){
		if(!this->receiveIsFirstMsg)
			ROS_WARN_STREAM("Cam \"" << cameraName
							<< "\" still has not received is first msg");
		else
			ROS_INFO_STREAM("Cam \"" << cameraName
							<< "\" has not respond for " << t.toSec() << "s");
	}
}

void ArtagSubscriber::artagCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg){
	if(!receiveIsFirstMsg)
		receiveIsFirstMsg = true;

	ROS_INFO_ONCE("Marker detected by camera");

	TrackedMarker trackMarkers = msg->markers;

	if(trackMarkers.empty()){
		ROS_INFO_STREAM("Cam \"" << cameraName << "\" no tag detected");
		// TODO: do something when too large
		emptyCount++;
		return;
	}
	else
		emptyCount = 0;

	tagsDetected.clear();

	Eigen::Matrix<double, 3, 4> cameraMatrixP;
	cameraMatrixP(0, 0) = msg->P[0];
	cameraMatrixP(0, 1) = msg->P[1];
    cameraMatrixP(0, 2) = msg->P[2];
    cameraMatrixP(0, 3) = msg->P[3];
    cameraMatrixP(1, 0) = msg->P[4];
    cameraMatrixP(1, 1) = msg->P[5];
    cameraMatrixP(1, 2) = msg->P[6];
    cameraMatrixP(1, 3) = msg->P[7];
    cameraMatrixP(2, 0) = msg->P[8];
    cameraMatrixP(2, 1) = msg->P[9];
    cameraMatrixP(2, 2) = msg->P[10];
    cameraMatrixP(2, 3) = msg->P[11];


	TrackedMarker::iterator m;
	for(m = trackMarkers.begin(); m != trackMarkers.end(); ++m){
		// Does the id is in the list of valid markers
		if(markers->isValidMarker(m->id)){
			TrackedMarker::iterator oldM;
			oldM = findMarkerInOldMsgById(m->id);
			// If the tag was not in the old msg
			if(oldM == oldMsg.markers.end()){
				ROS_INFO_STREAM("Cam \"" << cameraName
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

			Eigen::Matrix4d worldToTag;
			// Get tag's transformation
			worldToTag = markers->get(m->id).getEigen().matrix();

			// Set all required constant for the particle filter
			tagHandle_t t;
//			t.ref.cube2Cam_T = cubeToCam.translation();
//			t.ref.cube2Cam_R = cubeToCam.linear();
//			t.ref.world2Tag_T = worldToTag.translation();
//			t.ref.world2Tag_R = world2Tag_R_mat;
			t.ref.cube2Cam_H = cube2Cam;
			t.ref.posTag_W = worldToTag;
			t.ref.proj = cameraMatrixP;
			for(int i = 0; i < 4; i++){
				t.ref.corners[i](0) = m->corners[2 * i];
				t.ref.corners[i](1) = m->corners[2 * i + 1];
			}
//			t.ref.corners[0](0) = 247.078581347000;	t.ref.corners[0](1) =213.816556995000;
//			t.ref.corners[1](0) = 446.024811679000;	t.ref.corners[1](1) =216.513126468000;
//			t.ref.corners[2](0) = 444.251833741000;	t.ref.corners[2](1) =25.6693131215000;
//			t.ref.corners[3](0) = 252.690849739000;	t.ref.corners[3](1) =23.3895247245000;

//			t.cam2Tag_T = camToTag.translation();
//			t.cam2Tag_R = camToTag.linear();
			tagsDetected.push_back(t);
			// Calcul likelihood for particle

			/*pf->calcLogLikelihood(camToTag.translation(),
			                     Eigen::Quaterniond(camToTag.linear()),
			                     ref);*/
		}
		else
			ROS_WARN_STREAM("Cam \"" << cameraName
							<< "\" invalid tag detected id=" << m->id);
	}

	oldMsg = *msg;
	lastReception = ros::Time::now();
}

TrackedMarker::iterator ArtagSubscriber::findMarkerInOldMsgById(unsigned int id){
	std::vector<ar_track_alvar_msgs::AlvarMarker>::iterator it;
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


void ArtagSubscriber::pullTagDetected(std::list<tagHandle_t>& tagList){
	// Transfer objet container to the container pass has argument
	tagList.splice(tagList.end(), tagsDetected);
}

unsigned int ArtagSubscriber::getNumberTagsDetected(){
	return tagsDetected.size();
}
