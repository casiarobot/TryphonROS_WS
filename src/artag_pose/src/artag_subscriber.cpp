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
	sub = nh.subscribe<ar_track_alvar_msgs::AlvarMarkers>(
								topic_name,
								10,
								&ArtagSubscriber::artagCallback,
								this
							);



	lookupCameraTf();
}

void ArtagSubscriber::lookupCameraTf(){
	tf::TransformListener listener;
	try{
		listener.waitForTransform("/cube", cameraName, ros::Time(0), ros::Duration(10.0));
		listener.lookupTransform("/cube", cameraName, ros::Time(0), cubeToCamTf);
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

void ArtagSubscriber::artagCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg){
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

			tf::Pose camToTagTf, worldToTagTf;
			geometry_msgs::Pose pose;
			tf::poseMsgToTF(m->pose.pose, camToTagTf);
			worldToTagTf = markers->get(m->id).getTf();

			pose = fromRelativePoseToGlobalTf(camToTagTf,
											  worldToTagTf);
			validTagPose.push_back(pose);
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


geometry_msgs::Pose ArtagSubscriber::fromRelativePoseToGlobalTf(const tf::Pose& camToTagTf,
													 const tf::Pose& worldToTagTf){
	Eigen::Affine3d camToTag, worldToTag, cubeToCam;
	tf::poseTFToEigen(camToTagTf, camToTag);
	tf::poseTFToEigen(worldToTagTf, worldToTag);
	// TODO: attribute should be eigen type instead of tf
	tf::poseTFToEigen(cubeToCamTf, cubeToCam);

	// Angle correction on the camToTag frame
	double roll, yaw, pitch;
	roll = -M_PI/2;
	yaw = M_PI/2;
	pitch = 0;
	Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitZ());
	Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitY());
	Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitX());

	//::Quaternion<double> q =  yawAngle * rollAngle;


	// Instead of using a rotation to fix the frame error
	// the xyz are swap to be in the same correct frame
	Eigen::Affine3d camToTagRect, rotationRect; // Rectify cam to tag tf
	rotationRect = yawAngle * rollAngle;

	Eigen::Matrix3d rot;
	Eigen::Vector3d transl;
	rot = rotationRect.linear() * camToTag.linear() * rotationRect.linear().transpose();
	transl = rotationRect.linear() * camToTag.translation();

	// For test we put a identity
	Eigen::Matrix3d i3;
	i3 << 1, 0, 0,
		  0, 1, 0,
		  0, 0, 1;

	// The rotation matrix provided by alvar track is always rotate by pi in roll and yaw.
	// The solution is to multiply by a matrix that correct the (0,0) and (1,1) element.
	Eigen::Matrix3d m;
	m <<-1, 0, 0,
		 0,-1, 0,
		 0, 0, 1;
	rot *= m;
	//camToTagRect.linear() = i3; // DEBUG
	camToTagRect.linear() = rot;

	camToTagRect.translation() = transl;

	//camToTagRect(0, 3) = camToTag(2, 3); // x_r = z
	//camToTagRect(1, 3) = camToTag(0, 3); // y_r = x
	//camToTagRect(2, 3) = camToTag(1, 3); // y_r = x
	//Eigen::Rotation3d rotationPart;
	//rotationPart = camToTag.rotation();
	//camToTagRect = q * camToTag;
	//camToTagRect.s

	Eigen::Affine3d cubeToTag;
	cubeToTag = cubeToCam * camToTagRect;

	Eigen::Affine3d tagToCube;
	tagToCube = cubeToTag.inverse();

	Eigen::Affine3d worldToCube, worldToCube2;
	worldToCube = tagToCube * worldToTag;
	worldToCube2 = worldToTag * tagToCube;

	// Conversion from eigen to publishable ros msg
	tf::Pose inbetween;
	geometry_msgs::Pose output;
	tf::poseEigenToTF(worldToCube2, inbetween);
	tf::poseTFToMsg(inbetween, output);

	ROS_INFO_STREAM(std::endl << "rotationRect: " << std::endl << rotationRect.matrix());
	ROS_INFO_STREAM(std::endl << "cam   -> Tag: " << std::endl << camToTag.matrix());
	//ROS_INFO_STREAM(std::endl << "cam   -> TagRect_with_swap_xyz:" << std::endl << camToTagRect.matrix());
	ROS_INFO_STREAM(std::endl << "cam   -> TagRect_with_Quad:" << std::endl << camToTagRect.matrix());
	ROS_INFO_STREAM(std::endl << "cube  -> Cam: " << std::endl << cubeToCam.matrix());
	ROS_INFO_STREAM(std::endl << "cube  -> Tag: " << std::endl << cubeToTag.matrix());
	ROS_INFO_STREAM(std::endl << "Tag   -> cube: " << std::endl << tagToCube.matrix());
	ROS_INFO_STREAM(std::endl << "world -> Tag: " << std::endl << worldToTag.matrix());
	ROS_INFO_STREAM(std::endl << "world -> cube: " << std::endl << worldToCube.matrix());
	ROS_INFO_STREAM(std::endl << "world -> cube2: " << std::endl << worldToCube2.matrix());

	/*tf::Pose rect;
	rect.setOrigin(tf::Vector3(0,0,0));
	//rect.setRotation(tf::Quaternion(0,0,0,1));// x,y,z,w
	rect.setRotation(tf::Quaternion(M_PI / 2.0, 0, M_PI / 2.0));// ypr

	// The cam->tag from the alvar_track msg is from the wrong reference
	tf::Pose camToTagRect = rect * camToTag;

	// cube->cam + cam->tag = cube->tag
	tf::Pose cubeToTag = cubeToCamTf * camToTagRect;
	//tf::Pose cubeToTag = getPoseComposition(camToTagRect, cubeToCamTf);
	// (cube->tag)' = tag->cube
	tf::Pose tagToCube = cubeToTag.inverse();
	// world->tag + tag->cube = world->cube
	//tf::Pose worldToCube = getPoseComposition(worldToTag, tagToCube);
	tf::Pose worldToCube = worldToTag * tagToCube;
	tf::Pose cubeToWorld = worldToTag.inverse() * cubeToTag;


	geometry_msgs::Transform printableMsg;

	tf::transformTFToMsg(cubeToCamTf, printableMsg);
	ROS_INFO_STREAM("cubeToCamTf: "   << std::endl << printableMsg);

	tf::transformTFToMsg(camToTag, printableMsg);
	ROS_INFO_STREAM("camToTag: "   << std::endl << printableMsg);
	tf::transformTFToMsg(camToTagRect, printableMsg);
	ROS_INFO_STREAM("camToTagRect: "   << std::endl << printableMsg);///

	tf::transformTFToMsg(cubeToTag, printableMsg);
	ROS_INFO_STREAM("cubeToTag: "   << std::endl << printableMsg);

	tf::transformTFToMsg(tagToCube, printableMsg);
	ROS_INFO_STREAM("tagToCube: "   << std::endl << printableMsg);

	tf::transformTFToMsg(worldToCube, printableMsg);
	ROS_INFO_STREAM("worldToCube: " << std::endl << printableMsg);

	tf::transformTFToMsg(cubeToWorld, printableMsg);
	ROS_INFO_STREAM("cubeToWorld: " << std::endl << printableMsg);//*/

	return output;
}

// TODO Put in utility?
tf::Pose ArtagSubscriber::getPoseComposition(const tf::Pose& start,
											 const tf::Pose& increment) {
		tf::Pose finalPose;
		finalPose.setOrigin(start.getOrigin() + increment.getOrigin());
		finalPose.setRotation(increment.getRotation()*start.getRotation());

		return finalPose;
	}

bool ArtagSubscriber::wasMsgReceiveSinceLastPull(){
	return msgReceiveSincePull;
}

Eigen::Vector3d ArtagSubscriber::pullAveragePose(){
	msgReceiveSincePull = false;
	return avgPose;
}
