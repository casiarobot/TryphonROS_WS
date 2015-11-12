#include "artag_pose/artag_pose_node.h"



ArtagPoseNode::ArtagPoseNode(ParticleFilter* ppf):
    nodeHandle("~"),
	startNode(ros::Time::now()),
	pf(ppf){

	nodeHandle.param<int>("hz", frequency, 8);

	nodeHandle.param<double>("marker_size", marker_size, 10.0);
	marker_size /= 100; // convert to meter

	//nodeHandle.param<int>("", numMarkers, 2);
	numMarkers = 2;
	ROS_INFO_STREAM("Number markers: " << numMarkers);

	markerConfig = new TfConfigLoader;

	// Dynamice reconfiguration
	dynamic_reconfigure::Server<artag_pose::ArtagPoseConfig>::CallbackType dynamicReconfigCallback;
	dynamicReconfigCallback = boost::bind(&ArtagPoseNode::dynamicParametersCallback, this, _1, _2);
	dynamicReconfigServer.setCallback(dynamicReconfigCallback);


	srand(time(NULL));
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
	// compass
	std::string topic("/192_168_10_242/compass");
//	subCompass = nodeHandle.subscribe<sensors::compass>(
//								topic,
//								10,
//								&ArtagPoseNode::compassCallback,
//								this
//							);

	// Get topic list from parameter
	//std::vector<std::string> camera_topics = loadCameraTopics();

	// Bypass the Yaml configuration for debugging
	std::vector<std::string> camera_topics;
	camera_topics.push_back("camera1");
	camera_topics.push_back("/192_168_10_242/artags/artag1/ar_pose_marker");
	//camera_topics.push_back("camera2");
	//camera_topics.push_back("/192_168_10_242/artags/artag2/ar_pose_marker");
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

void ArtagPoseNode::compassCallback(const sensors::compass::ConstPtr& msg){
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

	unsigned int nbrCamera1Tag = 0, nbrCamera2Tag = 0;
	nbrCamera1Tag = artagSubs[0]->getNumberTagsDetected();
	if(artagSubs.size() > 1)
		nbrCamera2Tag = artagSubs[1]->getNumberTagsDetected();

	for(it = artagSubs.begin(); it != artagSubs.end(); ++it){
		(*it)->pullTagDetected(tagsDetected);
	}
	//nbrCamera2Tag = artagSubs[1]->getNumberTagsDetected();
	if(nbrCamera1Tag + nbrCamera2Tag < 1)
		return;

	//hardcodeValue1cam(tagsDetected);
	hardcodeValue2cam(tagsDetected, nbrCamera1Tag, nbrCamera2Tag);

	struct timeval tm1, tm2;
	gettimeofday(&tm1, NULL);
	for(int i = 0; i < 1; ++i){
		pf->updateParticle();
		pf->calcLogLikelihood(tagsDetected);
		pf->resampleParticles();
	}
	pf->calcLogLikelihood(tagsDetected);

	gettimeofday(&tm2, NULL);
	unsigned long long timel = 1000 * (tm2.tv_sec - tm1.tv_sec) + (tm2.tv_usec - tm1.tv_usec) / 1000;

	ROS_INFO_STREAM("Time for the particle update: " << timel << "ms, n =" << tagsDetected.size());
	//exit(0);

	// Publish visualization of the particle for Rviz
	geometry_msgs::PoseArray msg = pf->getParticleMsg();
	geometry_msgs::PoseStamped  msgBestLL = pf->getBestLikelihoodMsg(tagsDetected.begin()->ref);
	msg.header.frame_id = "cafeteria";
	msgBestLL.header.frame_id = "cafeteria";
	pubParticles.publish(msg);
	pubBestLLParticles.publish(msgBestLL);
	exit(0);
}

void ArtagPoseNode::hardcodeValue2cam(std::list<tagHandle_t> &tagsDetected, unsigned nb1, unsigned nb2){
	// Hard coded fake camera, with different reference
	std::list<tagHandle_t>::iterator t = tagsDetected.begin();
	// First tag
	for(int i = 0; i < nb1; i++){
		//t->cam2Tag_T = Eigen::Vector3d(1, 1, 0);

		t->ref.cube2Cam_H <<
		                     0,	0,	-1,	0, // cam2
							 1,	0,	0,	0,
							 0,	-1,	0,	0,
							 0,	0,	0,	1;
//		                     1,  0,  0,  0,//cam 1
//					         0,  0,  1,  0,
//		                     0, -1,  0,  0,
//					         0,  0,  0,  1;
		//TODO make it changeable

		double d = 0.10;
		Eigen::Matrix4d tagA;
		tagA << -d/2,  d/2,  d/2,  -d/2,
		         0,  0,  0,  0,
		         -d/2,  -d/2,  d/2,  d/2,
		         1,  1,  1,  1;
		Eigen::Vector3d tagPosA;
		//tagPosA << -0.0157, 0.3202, 0.0593;
		tagPosA << 0, 0, 0;
		t->ref.posTag_W = Eigen::Matrix4d::Identity(4,4);
		t->ref.posTag_W.col(3).topRows(3) = tagPosA;
		t->ref.posTag_W = t->ref.posTag_W*tagA;


		/*t->ref.posTag_W <<
//		                   0.3202,	0.3202,	0.3202,	0.3202, // cam2
//						   -0.0343,	0.0657,	0.0657,	-0.0343,
//						   0.0093,	0.0093,	0.1093,	0.1093,
//						   1,	1,	1,	1;
		                  -0.0657,	0.0343,	0.0343,	-0.0657, //Cam1
						   0.3202,	0.3202,	0.3202,	 0.3202,
						   0.0093,	0.0093,	0.1093,	 0.1093,
						   1,		1,		1,		 1;*/

		++t;
	}

	//Second tag
	for(int i = 0; i < nb2; i++){
		t->ref.cube2Cam_T = Eigen::Vector3d(-0.1, -0.1, 0);
		t->ref.cube2Cam_R <<
			   -1,  0,  0,
				0,  0, -1,
				0, -1,  0;
//			 1,  0,  0, // Facing right side
//			 0,  0,  1,
//			 0, -1,  0;

		//tag.ref.world2Tag_T = Eigen::Vector3d(0, -12.25, -0.5);
		t->ref.world2Tag_T = Eigen::Vector3d(-0.1, -0.63, 0);

		for(int i = 0; i < 4; i++){
			t->ref.world2Tag_T_corners[i] = t->ref.world2Tag_T;
		}
		t->ref.world2Tag_T_corners[2](0) -= marker_size/2;
		t->ref.world2Tag_T_corners[2](2) -= marker_size/2;
		t->ref.world2Tag_T_corners[3](0) += marker_size/2;
		t->ref.world2Tag_T_corners[3](2) -= marker_size/2;
		t->ref.world2Tag_T_corners[0](0) += marker_size/2;
		t->ref.world2Tag_T_corners[0](2) += marker_size/2;
		t->ref.world2Tag_T_corners[1](0) -= marker_size/2;
		t->ref.world2Tag_T_corners[1](2) += marker_size/2;

		//calculateCorners(t->ref, false, true);
//		t->ref.world2Tag_R <<
//			   -1,  0,  0,
//				0,  0,  1,
//				0,  1,  0;

		++t;
	}
}
void ArtagPoseNode::calculateCorners(tagRef_t &t, bool inverse_x, bool perp_to_y){
	for(int i = 0; i < 4; i++){
		t.world2Tag_T_corners[i](1) = ((i&1)^((i&2)>>1) == 1 ? 1 : -1) * marker_size/2;
		t.world2Tag_T_corners[i](2) = (i / 2 == 0 ? 1 : -1) * marker_size/2;
		if(inverse_x){
			std::swap(t.world2Tag_T_corners[i](0), t.world2Tag_T_corners[i](2));
		}
		if(perp_to_y){
			t.world2Tag_T_corners[i](0) *= -1;
			t.world2Tag_T_corners[i](2) *= -1;
		}
		t.world2Tag_T_corners[i](0) += t.world2Tag_T(0);
		t.world2Tag_T_corners[i](1) = t.world2Tag_T(1);
		t.world2Tag_T_corners[i](2) += t.world2Tag_T(2);

		ROS_INFO_STREAM(std::endl << "t.world2Tag_T_corners[i] "<< i << std::endl << t.world2Tag_T_corners[i]);
	}

}

void ArtagPoseNode::hardcodeValue1cam(std::list<tagHandle_t> &tagsDetected){
	// Hard coded fake camera, with different reference
	std::list<tagHandle_t>::iterator t = tagsDetected.begin();
	tagHandle_t tag = *t;
	// First tag
	//t->cam2Tag_T = Eigen::Vector3d(1, 1, 0);
	t->ref.cube2Cam_T = Eigen::Vector3d(0, 0, 0);
	t->ref.cube2Cam_R <<
	        0,  0,  1,
	       -1,  0,  0,
	        0, -1,  0;
	// inverse
//		0, -1,  0,
//		0,  0, -1,
//		1,  0,  0;
	//t->ref.world2Tag_T = Eigen::Vector3d(12.25, 0, -0.5);
	t->ref.world2Tag_T = Eigen::Vector3d(1, 0, 0);
	t->ref.world2Tag_R <<
	        0,  0, -1,
	       -1,  0,  0,
	        0,  1,  0;
	//Second tag
	//tag.cam2Tag_T = Eigen::Vector3d(1, 1, 0);
	tag.ref.cube2Cam_T = Eigen::Vector3d(0, 0, 0);
	tag.ref.cube2Cam_R <<
	       -1,  0,  0,
	        0,  0, -1,
	        0, -1,  0;
	// inversed
//	-1,  0,  0,
//     0,  0, -1,
//     0, -1,  0;
	//tag.ref.world2Tag_T = Eigen::Vector3d(0, -12.25, -0.5);
	tag.ref.world2Tag_T = Eigen::Vector3d(0, -1, 0);
	tag.ref.world2Tag_R <<
	       -1,  0,  0,
	        0,  0,  1,
	        0,  1,  0;

	//Add random camera lost after 10 secondes
	//if(ros::Time::now() - startNode > ros::Duration(10.0)
	//   && rand() % 2 == 0)
	//	return;

	tagsDetected.push_back(tag);
}


void ArtagPoseNode::dynamicParametersCallback(artag_pose::ArtagPoseConfig &config, uint32_t level){
	ROS_INFO("Parameters changed");

	pf->updateParameters(config.std_pose,
	                     config.std_T,
	                     config.std_R,
	                     config.std_DT,
	                     config.std_DR);
}


int main(int argc, char **argv){

	ros::init(argc, argv, "artag_pose_node");



	// Test for particle filter...
	// TODO remove this:
	double dt = 0;
	Eigen::MatrixXd forces(8,8);
	forces << 1, 0, 0, 0,dt, 0, 0, 0,
	          0, 1, 0, 0, 0,dt, 0, 0,
	          0, 0, 1, 0, 0, 0,dt, 0,
	          0, 0, 0, 1, 0, 0, 0,dt,
	          0, 0, 0, 0, 1, 0, 0, 0,
	          0, 0, 0, 0, 0, 1, 0, 0,
	          0, 0, 0, 0, 0, 0, 1, 0,
	          0, 0, 0, 0, 0, 0, 0, 1;
	// Range zero => zero at initiation
	Eigen::VectorXd range(8);
	range << 20, 20, 20, 60, 0, 0, 0, 0;
    int nbr_particles = 3;
    double std_pose = 30;
    double std_R = 0.25;
    double std_T = 0.03;

	Eigen::Matrix4d world2TagInit_H;
	world2TagInit_H <<
	                   1,	0,	0,	0,
					   0,	1,	0,	-1,
					   0,	0,	1,	0,
					   0,	0	,0	,1;
//	                      1,  0,  0,  -1.0,// old
//				          0,  1,  0,  0,
//	                      0,  0,  1,  0,
//				          0,  0,  0,  1;

	ParticleFilter *p = new ParticleFilter(forces, range, world2TagInit_H, nbr_particles, std_pose, std_R, std_T);

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
