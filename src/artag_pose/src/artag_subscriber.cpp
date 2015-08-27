#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
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
								10,
								&ArtagSubscriber::artagCallback,
								this
							);



	lookupCameraTf();
}

void ArtagSubscriber::lookupCameraTf(){
	tf::TransformListener listener;
	tf::StampedTransform cubeToCamTf;
	try{
		listener.waitForTransform("/cube", cameraName, ros::Time(0), ros::Duration(10.0));
		listener.lookupTransform("/cube", cameraName, ros::Time(0), cubeToCamTf);
		tf::poseTFToEigen(cubeToCamTf, cubeToCam);
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
			ROS_INFO_STREAM("Cam \"" << cameraName
							<< "\" has not respond for " << t.toSec() << "s");
		else
			ROS_WARN_STREAM("Cam \"" << cameraName
							<< "\" still has not received is first msg");
	}
}

void ArtagSubscriber::artagCallback(const ar_track_alvar::AlvarMarkers::ConstPtr& msg){
	if(!receiveIsFirstMsg)
		receiveIsFirstMsg = true;

	ROS_INFO_ONCE("Marker detected by camera");

	TrackedMarker trackMarkers = msg->markers;

	if(trackMarkers.empty()){
		ROS_INFO_STREAM("Cam \"" << cameraName
						<< "\" no tag detected");
		emptyCount++;
		return;
	}
	else
		emptyCount = 0;

	std::vector<geometry_msgs::Pose> validTagPose;
	TrackedMarker::iterator m;
	for(m = trackMarkers.begin(); m != trackMarkers.end(); ++m)
	for(unsigned i = 0; i < trackMarkers.size(); i++){
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

			Eigen::Affine3d camToTag, worldToTag;
			tf::poseMsgToEigen(m->pose.pose, camToTag);
			worldToTag = markers->get(m->id).getEigen();

			Eigen::Affine3d globalPose = fromRelativePoseToGlobalTf(camToTag, worldToTag);

			tagHandle newTag;
			newTag.camPose = camToTag;
			newTag.globalPose = globalPose;
			newTag.confidence = m->confidence;
			newTag.weight = 1.0;
			tagsDetected.push_back(newTag);
		}
		else
			ROS_WARN_STREAM("Cam \"" << cameraName
							<< "\" invalid tag detected id=" << m->id);
	}

	// If we receive valid pose, the main loop can retrieve it
	if(!validTagPose.empty()){
		lastReception = ros::Time::now();
		msgReceiveSincePull = true;
	}

	std::vector<geometry_msgs::Pose>::iterator it;
	for(it = validTagPose.begin(); it != validTagPose.end(); ++it){
		avgPose = *it;
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


	Eigen::Affine3d ArtagSubscriber::fromRelativePoseToGlobalTf(const Eigen::Affine3d& camToTag,
	                                                            const Eigen::Affine3d& worldToTag){

	// The yaw is directly extracted from the rotation matrix by transforming a
	// unit vector. Y plan seem to correspond to the yaw in simulation tests
	Eigen::Vector3d vrot = camToTag.linear() * Eigen::Vector3d::UnitX();
	double artagYaw = atan2(vrot(2), vrot(0));
	// The extracted yaw is than replace the rotation provided by alvar
	// In ros the yaw is projected on the Z plan
	Eigen::Matrix3d yawRotationMat;
	yawRotationMat = Eigen::AngleAxisd(artagYaw, Eigen::Vector3d::UnitZ());

	// Angle correction on the camToTag frame
	double roll, yaw, pitch;
	// yaw = pi/2 and roll -pi/2 fix the xyz swap from the artag frame to the camera frame
	yaw =   M_PI/2; //correct artag
	roll = -M_PI/2;
	//pitch = M_PI/2;
	Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitZ());
	Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitY());
	//Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitX());

	Eigen::Affine3d camToTagRect, artagRect, rotationRect; // Rectify cam to tag tf
	//artagRect = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());
	rotationRect = yawAngle * rollAngle;

	//camToTagRect.linear() =  artagRect.linear() * camToTag.linear();
	camToTagRect.linear() =  yawRotationMat; // Ignore the original rotation provided by alvar
	camToTagRect.translation() =  rotationRect.linear() * camToTag.translation(); // Does xyz swap


	Eigen::Affine3d cubeToTag;
	cubeToTag = cubeToCam * camToTagRect;

	Eigen::Affine3d tagToCube;
	tagToCube = cubeToTag.inverse();

	Eigen::Affine3d worldToCube, worldToCube2, worldToCube3;
	worldToCube = tagToCube * worldToTag;
	worldToCube2 = worldToTag * tagToCube;
	worldToCube3.linear() =  worldToTag.linear() * tagToCube.linear();
	worldToCube3.translation() =  tagToCube.translation() + worldToTag.translation();

	// Conversion from eigen to publishable ros msg
	//tf::Pose inbetween;
	//geometry_msgs::Pose output;
	//tf::poseEigenToTF(worldToCube2, inbetween);
	//tf::poseTFToMsg(inbetween, output);

	ROS_INFO_STREAM(std::endl << "artagYaw: " << artagYaw * 180.0 / M_PI);
	ROS_INFO_STREAM(std::endl << "rotationRect: " << std::endl << rotationRect.matrix());
	ROS_INFO_STREAM(std::endl << "cam   -> Tag: " << std::endl << camToTag.matrix());
	//ROS_INFO_STREAM(std::endl << "cam   -> TagRect_with_swap_xyz:" << std::endl << camToTagRect.matrix());
	ROS_INFO_STREAM(std::endl << "cam   -> TagRect_with_Quad:" << std::endl << camToTagRect.matrix());
	//ROS_INFO_STREAM(std::endl << "cam   -> TagRect_mul: " << std::endl << camToTagRect_mul.matrix());
	ROS_INFO_STREAM(std::endl << "cube  -> Cam: " << std::endl << cubeToCam.matrix());
	ROS_INFO_STREAM(std::endl << "cube  -> Tag: " << std::endl << cubeToTag.matrix());
	ROS_INFO_STREAM(std::endl << "Tag   -> cube: " << std::endl << tagToCube.matrix());
	ROS_INFO_STREAM(std::endl << "world -> Tag: " << std::endl << worldToTag.matrix());
	ROS_INFO_STREAM(std::endl << "world -> cube: " << std::endl << worldToCube.matrix());
	ROS_INFO_STREAM(std::endl << "world -> cube2: " << std::endl << worldToCube2.matrix());
	//ROS_INFO_STREAM(std::endl << "world -> cube3_transpose: " << std::endl << worldToCube3.matrix());
	Eigen::Quaternion<double> q;
	q = worldToCube2.linear().matrix();
	ROS_INFO_STREAM("to Quaternion [x y z w]:" << std::endl <<
												q.x() << std::endl <<
												q.y() << std::endl <<
												q.z() << std::endl <<
												q.w() << std::endl);
	Eigen::Vector3d ea = worldToCube2.linear().matrix().eulerAngles(0, 1, 2) * 180.0/M_PI;
	ROS_INFO_STREAM("to Euler angles:" << std::endl << ea << std::endl << std::endl);

	// Tf broadcast for debugging:
	tf::Transform camToTagBroadcast;
	tf::poseEigenToTF(camToTagRect, camToTagBroadcast);
	br.sendTransform(tf::StampedTransform(camToTagBroadcast, ros::Time::now(), cameraName, "debug_artag"));

	return worldToCube2;
}

// TODO Put in utility?
tf::Pose ArtagSubscriber::getPoseComposition(const tf::Pose& start,
                                             const tf::Pose& increment) {
	tf::Pose finalPose;
	finalPose.setOrigin(start.getOrigin() + increment.getOrigin());
	finalPose.setRotation(increment.getRotation()*start.getRotation());

	return finalPose;
}

bool ArtagSubscriber::receivedMsgSinceLastPull(){
	return msgReceiveSincePull;
}

void ArtagSubscriber::pullAveragePose(std::list<tagHandle>& tagList){
	msgReceiveSincePull = false;

	// Transfer objet container to the container pass has argument
	tagList.splice(tagList.end(), tagsDetected);
}
