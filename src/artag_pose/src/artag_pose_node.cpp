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
	//hardcodeValue2cam(tagsDetected, nbrCamera1Tag, nbrCamera2Tag);

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
}

void ArtagPoseNode::hardcodeValue2cam(std::list<tagHandle_t> &tagsDetected, unsigned nb1, unsigned nb2){
	// Hard coded fake camera, with different reference
	std::list<tagHandle_t>::iterator t = tagsDetected.begin();
	// First tag
	//for(int i = 0; i < nb1; i++){
		//t->cam2Tag_T = Eigen::Vector3d(1, 1, 0);

		t->ref.cube2Cam_H <<
//		                     0,	0,	-1,	0, // cam2
//							 1,	0,	0,	0,
//							 0,	-1,	0,	0,
//							 0,	0,	0,	1;
		                     1,  0,  0,  0,//cam 1
					         0,  0,  1,  0,
		                     0, -1,  0,  0,
					         0,  0,  0,  1;
		//TODO make it changeable

		double d = 0.10;
		Eigen::Matrix4d tagA;
		tagA << -d/2,  d/2,  d/2,  -d/2,
		         0,  0,  0,  0,
		         -d/2,  -d/2,  d/2,  d/2,
		         1,  1,  1,  1;
		Eigen::Vector3d tagPosA;
		//tagPosA << -0.0157, 0.3202, 0.0593;
		tagPosA << 0.0157, 0.3202, 0.0593;
		t->ref.posTag_W = Eigen::Matrix4d::Identity(4,4);
		t->ref.posTag_W.col(3).topRows(3) = tagPosA;
		t->ref.posTag_W = t->ref.posTag_W * tagA;

		//ROS_INFO_STREAM("dude it's late"<<std::endl << t->ref.posTag_W);


//		t->ref.posTag_W <<
//		                   0.3202,	0.3202,	0.3202,	0.3202, // cam2
//						   -0.0343,	0.0657,	0.0657,	-0.0343,
//						   0.0093,	0.0093,	0.1093,	0.1093,
//						   1,	1,	1,	1;
//		                  -0.0657,	0.0343,	0.0343,	-0.0657, //Cam1
//						   0.3202,	0.3202,	0.3202,	 0.3202,
//						   0.0093,	0.0093,	0.1093,	 0.1093,
//						   1,		1,		1,		 1;

	//	++t;
	//}
	//Second tag
	tagHandle_t t2 = *t;

	t2.ref.cube2Cam_H <<
		                     0,	0,	1,	0, // cam2
							 -1,0,	0,	0,
							 0,	-1,	0,	0,
							 0,	0,	0,	1;
	Eigen::Matrix3d rot90;
	rot90 <<  0,    1,    0,
	         -1,    0,    0,
		      0,    0 ,   1;
	t2.ref.posTag_W = Eigen::Matrix4d::Identity(4,4);
	t2.ref.posTag_W.block(0, 0, 3, 3) = rot90;
	Eigen::Vector3d tagPosB(0.3202, -0.0157, 0.0593);
	t2.ref.posTag_W.col(3).topRows(3) = tagPosB;
	t2.ref.posTag_W = t2.ref.posTag_W * tagA;
//	t2.ref.posTag_W <<
//	0.3202,	0.3202,	0.3202,	0.3202,
//	0.0343,	-0.0657,-0.0657, 0.0343,
//	0.0093,	0.0093,	0.1093,	0.1093,
//	1,	1,	1,	1;

	tagsDetected.push_back(t2);



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
	range << 20, 20, 20, 170, 0, 0, 0, 0;

    int nbr_particles = 300;
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
