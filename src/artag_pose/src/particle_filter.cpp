#include "artag_pose/particle_filter.h"

using namespace std;

ParticleFilter::ParticleFilter(
				const Eigen::MatrixXd& pForces,
				const Eigen::VectorXd& pRange,
                int pNbrParticles,
                double pStdPose,
                double pStdR,
                double pStdT):
    forces(pForces),
    range(pRange),
    nbr_particles(pNbrParticles),
	std_pose(pStdPose),
	std_R(pStdR),
	std_T(pStdT),
    indexMaxLikelihood(0)
{
	//init standard librairy seed initiation
	srand((unsigned int) time(0));

	createParticles();
}

// Generate the particle with uniform random value in the range
void ParticleFilter::createParticles(){
	// Fill the array with random value from -1 to 1
	this->particles = Eigen::MatrixXd::Random(NBR_PARAMETERS, nbr_particles);

	// Set each parameter to the correction range
	// for -20 to 20, we multiply by 20 the -1 to 1 random number to get a -20 to 20
	// TODO: Add an actual range instead of this hack
	for(int i = 0; i < NBR_PARAMETERS; ++i){
		particles.row(i) *= range(i);
	}
	//ROS_INFO_STREAM(std::endl << "Particles initiation value: " << std::endl << particles);

	// Create likelihood vector
	ll = Eigen::VectorXd(nbr_particles);
}

/*
void ParticleFilter::update(const Eigen::Vector3d &translation, const Eigen::Quaterniond &rotation){
	ROS_INFO_STREAM(std::endl << "cam2tag translation: " << std::endl << translation);
	printQuaternion(string("cam2tag rotation: "), rotation);

	updateParticle();
	ROS_INFO_STREAM(std::endl << "Particles updated value: " << std::endl << particles);
	calcLogLikelihood(translation, rotation);
	resampleParticles();
	ROS_INFO_STREAM(std::endl << "Particle end: " << std::endl << particles);
}*/

void ParticleFilter::updateParticle(){
	// Apply the forces
	particles = forces * particles;

	// Add random value on the Translation parameter (the first 3 parameters)
	particles.topRows(3) += std_T * Eigen::MatrixXd::Random(3, nbr_particles);
	// Fourth row
	particles.row(3) += std_R * Eigen::MatrixXd::Random(1, nbr_particles);

	// Update rows 5 to 7
	particles.bottomRows(4).topRows(3) += std_T / 5.0 * Eigen::MatrixXd::Random(3, nbr_particles);
	// 8th row
	particles.row(7) += std_R / 5.0 * Eigen::MatrixXd::Random(1, nbr_particles);

}

void ParticleFilter::calcLogLikelihood(const  std::list<tagHandle_t> &tags,
                                       const bool reculsive_flag){
	//double A = -log(sqrt(2* M_PI) * std_pose);
	//double B = -0.5 / (std_pose * std_pose);
	std::list<tagHandle_t>::const_iterator it;

	bool oneParticleInRange = false;
	if(reculsive_flag)
		oneParticleInRange  = true;

	for(int k = 0; k < nbr_particles; ++k){

		bool inRange = true;
		for(int p = 0; p < 4; ++p){
			if(abs(particles(p, k)) > range(p)){
				inRange = false;
			}
		}

		// If 4 first parameters are in range
		if(inRange){
			oneParticleInRange = true;
			double yaw = particles(3, k);
			Eigen::Matrix3d world2Cube_R_guess;
			world2Cube_R_guess = Eigen::AngleAxisd(-yaw * M_PI/ 180.0, Eigen::Vector3d::UnitZ());
			//world2Cube_R_guess = Eigen::AngleAxisd(-0 * M_PI/ 180.0, Eigen::Vector3d::UnitZ());

			const Eigen::Vector3d T_guess = particles.col(k).topRows(3);

			for(it = tags.begin(); it != tags.end(); ++it){
				Eigen::Vector3d error =
						(world2Cube_R_guess * it->ref.cube2Cam_R).inverse()
						* (it->ref.world2Tag_T -T_guess) -it->ref.cube2Cam_R * it->ref.cube2Cam_T
						-it->cam2Tag_T;
				double D = error.transpose() * error;

				double stdPose = it->cam2Tag_T.norm() * std_pose;
				double A = -log(sqrt(2* M_PI) * stdPose);
				double B = -0.5 / (stdPose * stdPose);
				ll(k) += A + B * D;
			}


		}
		else{
			ll(k) = -std::numeric_limits<double>::max();// = -inf
		}
	}
	// Every particles is of infinite likelihood
	if(!oneParticleInRange){
		createParticles();
		updateParticle();
		calcLogLikelihood(tags, true);
		ROS_ERROR_STREAM("RESET OF THE PARTICLE!!!");
	}

}

// Shameless copy from https://github.com/libigl/libigl/blob/89a8257d152165218fc514e4e35a5aa56dfeca10/include/igl/histc.cpp
inline void histc(
  const Eigen::VectorXd & X,
  const Eigen::VectorXd & E,
  Eigen::VectorXd & B)
{
  const int m = X.size();
  using namespace std;

  for(int j = 0;j<m;j++)
  {
    const double x = X(j);
    // Boring one-offs
    if(x < E(0) || x > E(E.size()-1))
    {
      B(j) = -1;
      continue;
    }
    // Find x in E
    int l = 0;
    int h = E.size()-1;
    int k = l;
    while((h-l)>1)
    {
      assert(x >= E(l));
      assert(x <= E(h));
      k = (h+l)/2;
      if(x < E(k))
      {
        h = k;
      }else
      {
        l = k;
      }
    }
    if(x == E(h))
    {
      k = h;
    }else
    {
      k = l;
    }
    B(j) = k;
  }
}


inline void histc(
  const Eigen::VectorXd & X,
  const Eigen::VectorXd & E,
  Eigen::VectorXd & N,
  Eigen::VectorXd & B)
{
  histc(X,E,B);
  const int n = E.size();
  const int m = X.size();
  assert(m == B.size());
  N.resize(n,1);
  N.setConstant(0);
  for(int j = 0;j<m;j++)
  {
    if(B(j) >= 0)
    {
      N(B(j))++;
    }
  }
}

void ParticleFilter::resampleParticles(){
	//ROS_INFO_STREAM(std::endl << "max:" << std::endl << ll.maxCoeff() );

	ROS_INFO_STREAM(std::endl << "ll" << std::endl << ll);

	ll = ll - ll.maxCoeff(&indexMaxLikelihood) * Eigen::VectorXd::Ones(nbr_particles);
	ll = ll.array().exp();
	Eigen::VectorXd Q = ll / ll.sum();
	Eigen::VectorXd R = Q;
	// Currently no equivalant to cumsum in Eigen
	for(int i = 1; i < nbr_particles; ++i)
		R(i) += R(i - 1);

	Eigen::VectorXd T = Eigen::VectorXd::Random(nbr_particles);

	Eigen::VectorXd ignore(nbr_particles), I(nbr_particles);
	histc(T, R, ignore, I);

	// Resampling, update particles with the
	Eigen::MatrixXd Y = particles;
	for(int i = 0; i < nbr_particles; ++i){
		Y.col(i) = particles.col(I(i)+1);
	}
	particles = Y;

}

geometry_msgs::PoseArray ParticleFilter::getParticleMsg(){
	// TODO init at resize this array
	geometry_msgs::PoseArray msgArray;
	geometry_msgs::Pose m;

	// Let's not draw all the particles...
	unsigned int n = nbr_particles;
	if(nbr_particles > 100)
		n = 100;
	msgArray.poses.reserve(n);
	for(int i = 0; i < n; ++i){
		m.position.x = particles(0, i);
		m.position.y = particles(1, i);
		m.position.z = particles(2, i);
		msgArray.poses.push_back(m);
	}

	return msgArray;
}

geometry_msgs::PoseStamped  ParticleFilter::getBestLikelihoodMsg(){
	// TODO init at resize this array
	geometry_msgs::PoseStamped m;
	double bllAngle = particles(3, indexMaxLikelihood);
	double maxRange = range(3)/10.0;
	double angle = 0;
	int n = 0;
	for(int i = 0; i < nbr_particles; ++i){
		// Check if the particle's angle is not far from the best likelihood's angle
		if(abs(bllAngle - particles(3, i)) < maxRange){
			m.pose.position.x += particles(0, i);
			m.pose.position.y += particles(1, i);
			m.pose.position.z += particles(2, i);
			angle += particles(3, i);
			n++;
		}
	}
	ROS_INFO_STREAM(n);
	m.pose.position.x /= (double)n;
	m.pose.position.y /= (double)n;
	m.pose.position.z /= (double)n;
	angle /= (double)n;

	Eigen::Quaterniond q;
	q = Eigen::AngleAxisd(-angle * M_PI/ 180.0, Eigen::Vector3d::UnitZ());
	m.pose.orientation.x = q.x();
	m.pose.orientation.y = q.y();
	m.pose.orientation.z = q.z();
	m.pose.orientation.w = q.w();

	return m;
}

void ParticleFilter::printQuaternion(const std::string title, const Eigen::Quaterniond &quat) const{
	 ROS_INFO_STREAM(std::endl << title << std::endl << quat.x() << std::endl  << quat.y() << std::endl  << quat.z() << std::endl  << quat.w());
}

void ParticleFilter::setStdPose(double s){
	std_pose = s;
}
