#include "artag_pose/artag_pose_node.h"



ArtagPoseNode::ArtagPoseNode(ParticleFilter* ppf):
    nodeHandle("~"),
	pf(ppf){

	nodeHandle.param<int>("hz", frequency, 10);

	nodeHandle.param<int>("numMarkers", numMarkers, 1);

	markerConfig = new TfConfigLoader;

	// Dynamice reconfiguration
	dynamic_reconfigure::Server<artag_pose::ArtagPoseConfig>::CallbackType dynamicReconfigCallback;
	dynamicReconfigCallback = boost::bind(&ArtagPoseNode::dynamicParametersCallback, this, _1, _2);
	dynamicReconfigServer.setCallback(dynamicReconfigCallback);
}

ArtagPoseNode::~ArtagPoseNode(){
	artagSubs.clear();
	delete markerConfig;
}


void ArtagPoseNode::start(){
	loadConfig();
	createPublishers();
	createSubscribers();
	loop();
}
void ArtagPoseNode::loadConfig(){
	markerConfig->load(numMarkers);
	markersPose = markerConfig->parse();
}

void ArtagPoseNode::createPublishers(){
	pubPose = nodeHandle.advertise<geometry_msgs::PoseStamped>("multitag_pose", 1000);
	pubParticles = nodeHandle.advertise<geometry_msgs::PoseArray>("particles_makers", 1000);
	pubBestLLParticles = nodeHandle.advertise<geometry_msgs::PoseStamped>("best_likelihood_maker", 1000);
}

void ArtagPoseNode::createSubscribers(){
	// Get topic list from parameter
	//std::vector<std::string> camera_topics = loadCameraTopics();

	// Bypass the Yaml configuration for debugging
	std::vector<std::string> camera_topics;
	camera_topics.push_back("camera1");
	camera_topics.push_back("/192_168_10_243/artags/artag1/ar_pose_marker");
	//camera_topics.push_back("camera2");
	//camera_topics.push_back("/192_168_10_242/artags/artag2/ar_pose_marker");
	//camera_topics.push_back("camera4");
	//camera_topics.push_back("/192_168_10_243/artags2/artag1/ar_pose_marker");
	/*camera_topics.push_back("camera3");
	camera_topics.push_back("/192_168_10_243/artags/artag3/ar_pose_marker");*/

	// for each camera topics create a subscriber
	std::vector<std::string>::iterator topic_name;
	for(topic_name = camera_topics.begin();
	    topic_name != camera_topics.end();
	    topic_name++){
		std::string camera_name = *topic_name;
		// The following element is the topic name
		topic_name++;
		ArtagSubPtr ptr(new ArtagSubscriber(camera_name, *topic_name, markersPose, nodeHandle, pf));
		artagSubs.push_back(ptr);
	}
}


std::vector<std::string> ArtagPoseNode::loadCameraTopics(){
	std::vector<std::string> camera_topics;

	bool parameter_provided = nodeHandle.getParam("camera_topics", camera_topics);

	if(!parameter_provided){
		ROS_ERROR("Can't load list of camera's topic from parameter \"camera_topics\"");
		ros::shutdown();
	}

	return camera_topics;
}

void ArtagPoseNode::loop(){
	ros::Rate loop_rate(frequency);
	while(ros::ok()){
		computePoseAndPublish();

		ros::spinOnce();
		loop_rate.sleep();
	}
}

void ArtagPoseNode::computePoseAndPublish(){
	std::vector<ArtagSubPtr>::iterator it;


	std::list<tagHandle_t> tagsDetected;

	//for(it = artagSubs.begin(); it != artagSubs.end(); ++it){
	//	(*it)->pullTagDetected(tagsDetected);
	//}
	int nbrCamera1Tag = 0, nbrCamera2Tag = 0;
	nbrCamera1Tag = artagSubs[0]->getNumberTagsDetected();
	//nbrCamera2Tag = artagSubs[1]->getNumberTagsDetected();
	if(nbrCamera1Tag + nbrCamera2Tag < 1)
		return;

	artagSubs[0]->pullTagDetected(tagsDetected);
	//artagSubs[1]->pullTagDetected(tagsDetected);



	// Hard coded fake camera, with different reference
	std::list<tagHandle_t>::iterator t = tagsDetected.begin();
	for(int i = 0; i < nbrCamera1Tag; i++){
		// First tag
		//t->cam2Tag_T = Eigen::Vector3d(1, 1, 0);
		t->ref.cube2Cam_T = Eigen::Vector3d(0, 0, 0);
		t->ref.cube2Cam_R <<
				0,  0,  1,
			   -1,  0,  0,
				0, -1,  0;
		t->ref.world2Tag_T = Eigen::Vector3d(1, 0, 0);
		t->ref.world2Tag_R <<
				0,  0, -1,
			   -1,  0,  0,
				0,  1,  0;
		//++t; // Incremente iterator
	}

	/*for(int i = 0; i < nbrCamera2Tag; i++){
		//tag.cam2Tag_T = Eigen::Vector3d(1, 1, 0);
		t->ref.cube2Cam_T = Eigen::Vector3d(0.7, -0.14, 0.15);
		t->ref.cube2Cam_R <<
		         0,  0,  1,
		        -1,  0,  0,
				 0, -1,  0;
	//	       -1,  0,  0,
	//	        0,  0, -1,
	//	        0, -1,  0;
		t->ref.world2Tag_T = Eigen::Vector3d(1, 0, 0);
		t->ref.world2Tag_R <<
			    0,  0, -1,
			   -1,  0,  0,
			    0,  1,  0;
	//	       -1,  0,  0,
	//	        0,  0,  1,
	//	        0,  1,  0;
		++t; // Incremente iterator
	}*/
	tagHandle_t tag = *t;
	tag.cam2Tag_T = Eigen::Vector3d(0, 0, 0);
	tag.ref.cube2Cam_R <<
//	         0,  0,  1, // front
//	        -1,  0,  0,
//			 0, -1,  0;
	       -1,  0,  0, // Left side
	        0,  0, -1,
	        0, -1,  0;
	tag.ref.world2Tag_T = Eigen::Vector3d(0, -1, 0);
	tag.ref.world2Tag_R <<
//		    0,  0, -1,
//		   -1,  0,  0,
//		    0,  1,  0;
	       -1,  0,  0,
	        0,  0,  1,
	        0,  1,  0;
	tagsDetected.push_back(tag);
	//tagsDetected.erase(tagsDetected.begin());
	struct timeval tm1;
	gettimeofday(&tm1, NULL);

	pf->updateParticle();
	//tagHandle_t t = *tagsDetected.begin();
	pf->calcLogLikelihood(tagsDetected, false);
	pf->resampleParticles();
	struct timeval tm2;
	gettimeofday(&tm2, NULL);


	unsigned long long timel = 1000 * (tm2.tv_sec - tm1.tv_sec) + (tm2.tv_usec - tm1.tv_usec) / 1000;

	ROS_INFO_STREAM("Time for the particle update: " << timel
	                << "ms, with n=" << nbrCamera1Tag + nbrCamera2Tag);

	// Publish visualization of the particle for Rviz
	geometry_msgs::PoseArray msg = pf->getParticleMsg();
	geometry_msgs::PoseStamped  msgBestLL = pf->getBestLikelihoodMsg();
	msg.header.frame_id = "cafeteria";
	msgBestLL.header.frame_id = "cafeteria";
	pubParticles.publish(msg);
	pubBestLLParticles.publish(msgBestLL);
}


void ArtagPoseNode::dynamicParametersCallback(artag_pose::ArtagPoseConfig &config, uint32_t level){
	ROS_INFO("Parameters changed");

	pf->setStdPose(config.std_pose);
}


int main(int argc, char **argv){

	ros::init(argc, argv, "artag_pose_node");



	// Test for particle filter...
	// TODO remove this:

	Eigen::MatrixXd forces(8,8);
	forces << 1, 0, 0, 0, 1, 0, 0, 0,
	          0, 1, 0, 0, 0, 1, 0, 0,
	          0, 0, 1, 0, 0, 0, 1, 0,
	          0, 0, 0, 1, 0, 0, 0, 1,
	          0, 0, 0, 0, 1, 0, 0, 0,
	          0, 0, 0, 0, 0, 1, 0, 0,
	          0, 0, 0, 0, 0, 0, 1, 0,
	          0, 0, 0, 0, 0, 0, 0, 1;
	// Range zero => zero at initiation
	Eigen::VectorXd range(8);
	range << 20, 20, 20, 60, 0, 0, 0, 0;
    int nbr_particles = 300; // 300
    double std_pose = 0.1;
    double std_R = 0.25;
    double std_T = 0.1;

	//Eigen::Vector3d cube2Cam_T(0,0,0);
	//Eigen::Vector3d world2Tag_T(12.25, 0, -0.5);
	Eigen::Matrix3d world2Tag_R_mat, cube2Cam_R_mat;
	world2Tag_R_mat << 0, 0, -1,
				      -1, 0,  0,
				       0, 1,  0;
	Eigen::Quaterniond world2Tag_R(world2Tag_R_mat);
	cube2Cam_R_mat << 0, -1,  0,
				      0,  0, -1,
				      1,  0,  0;
	//Eigen::AngleAxisd test;
	//test.fromRotationMatrix(cube2Cam_R_mat);
	Eigen::Quaterniond cube2Cam_R(cube2Cam_R_mat);

	ROS_INFO_STREAM(std::endl << "cube2Cam_R: " << std::endl << cube2Cam_R.toRotationMatrix());

	ParticleFilter *p = new ParticleFilter(forces, range, nbr_particles, std_pose, std_R, std_T);

	ArtagPoseNode mw(p);

	mw.start();
	/*
	// TODO load a csv
	Eigen::MatrixXd data(4,7);
	// tx, ty, tz, rx, ry, rz, rw
	data << 0.131816708409,0.763523402815,12.9119347279,0.997427140486,-0.00453556586724,-0.00616969498657,0.0712773661727,
	        0.131931847866,0.763413953575,12.9100207,0.997298885456,-0.00455192282082,-0.00803591208295,0.0728672572854,
	        0.131973728663,0.763381963781,12.909917379,0.997402676848,-0.00421134000945,-0.00476821825192,0.0717455847183,
	        0.13227620295,0.762760021493,12.9041169895,0.997132343074,-0.00376080959683,0.00331893837823,0.0755111339748;
	Eigen::Vector3d cam2Tag_T;
	Eigen::Quaterniond cam2Tag_R;
	int nbr_iter = 2;
	for(int  i = 0; i < nbr_iter; i++){
			cam2Tag_T = Eigen::Vector3d(data(i, 0), data(i, 1), data(i, 2));
			cam2Tag_R = Eigen::Quaterniond(data(i, 6), data(i, 3), data(i, 4), data(i, 5)); // Eigen is q(w, x, y, z)
			p.update(cam2Tag_T, cam2Tag_R);
	}

	*/
	return 0;
}
