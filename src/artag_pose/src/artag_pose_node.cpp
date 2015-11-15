#include "artag_pose/artag_pose_node.h"



ArtagPoseNode::ArtagPoseNode(ParticleFilter* ppf):
    nodeHandle("~"),
	startNode(ros::Time::now()),
	pf(ppf),
	initializing(true),
	settingFirstOffset(false),
	offset(0,0,0){

	nodeHandle.param<int>("hz", frequency, 8);

	nodeHandle.param<double>("marker_size", marker_size, 10.0);
	marker_size /= 100; // convert to meter

	//nodeHandle.param<int>("", numMarkers, 2);
	numMarkers = 3;
	ROS_INFO_STREAM("Number markers: " << numMarkers);

	//markerConfig = new TfConfigLoader;
	markerConfig = new DefaultConfig;

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

	Eigen::Matrix4d cube2Cam_H;
	cube2Cam_H <<
	              0,	0,	1,	0, // cam2
				 -1,	0,	0,	0,
				  0,   -1,	0,	0,
				  0,	0,	0,	1;
	// Camera on the right side
	ArtagSubPtr ptr1(new ArtagSubscriber("camera1",
	                                    "/192_168_10_242/artags/artag1/ar_pose_marker",
	                                    cube2Cam_H,
	                                    markersPose,
	                                    nodeHandle,
	                                    pf));
	artagSubs.push_back(ptr1);
	cube2Cam_H <<
				 1,  0,  0,  0,//cam 1
				 0,  0,  1,  0,
				 0, -1,  0,  0,
				 0,  0,  0,  1;
	// Camera at the front
	cube2Cam_H.col(3).topRows(3) = Eigen::Vector3d(1, -1.0, 0);
	ArtagSubPtr ptr2(new ArtagSubscriber("camera2",
	                                    "/192_168_10_242/artags/artag2/ar_pose_marker",
	                                    cube2Cam_H,
	                                    markersPose,
	                                    nodeHandle,
	                                    pf));
	artagSubs.push_back(ptr2);
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
	ROS_WARN("Initializing...");
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

	if(initializing){
		if(nbrCamera1Tag + nbrCamera2Tag >= numMarkers){
			markerConfig->init(tagsDetected);
			initializing = false;
			settingFirstOffset = true;
		}
		else{
			ROS_INFO_STREAM("Initializing, detected " << tagsDetected.size() << "/" << numMarkers);
		}
		// Paramater during the initiation phase
		pf->updateParameters(30.0,
		                     0.06,
		                     0.4);
		return; // We wait another cycle
	}


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
	geometry_msgs::PoseArray msg = pf->getParticleMsg(offset);
	geometry_msgs::PoseStamped  msgBestLL = pf->getBestLikelihoodMsg(offset);

	double var = pf->getVariance();
	if(settingFirstOffset && var < 0.5){
		offset(0) = -msgBestLL.pose.position.x;
		offset(1) = -msgBestLL.pose.position.y;
		offset(2) = -msgBestLL.pose.position.z;
		settingFirstOffset = false;
		ROS_INFO_STREAM("(N)Var =" << var);
	}
	else{
		pf->updateParameters(std_pose,
		                     std_T,
		                     std_R);
		ROS_INFO_STREAM("(O)Var =" << var);
	}


	msg.header.frame_id = "cafeteria";
	msgBestLL.header.frame_id = "cafeteria";
	pubParticles.publish(msg);
	pubBestLLParticles.publish(msgBestLL);
}



void ArtagPoseNode::dynamicParametersCallback(artag_pose::ArtagPoseConfig &config, uint32_t level){
	ROS_INFO("Parameters changed");
	std_pose = config.std_pose;
	std_T = config.std_T;
	std_R = config.std_R;
	pf->updateParameters(config.std_pose,
	                     config.std_T,
	                     config.std_R);
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
	range << 20, 20, 20, 170, 0, 0, 0, 0;

    int nbr_particles = 300;
    double std_pose = 30;
    double std_R = 0.25;
    double std_T = 0.03;

	Eigen::Matrix4d world2TagInit_H;
	world2TagInit_H <<
	                   1,	0,	0,	-0.5,
					   0,	1,	0,	-0.5,
					   0,	0,	1,	-0.5,
					   0,	0	,0	,1;

	ParticleFilter *p = new ParticleFilter(forces, range, world2TagInit_H, nbr_particles, std_pose, std_R, std_T);

	ArtagPoseNode mw(p);

	mw.start();
	return 0;
}
