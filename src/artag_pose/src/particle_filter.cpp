#include "artag_pose/particle_filter.h"

using namespace std;

ParticleFilter::ParticleFilter(
				const Eigen::MatrixXd& pForces,
				const Eigen::VectorXd& pRange,
				const Eigen::Matrix4d& pWorld2TagInit_H,
                int pNbrParticles,
                double pStdPose,
                double pStdR,
                double pStdT):
    forces(pForces),
    range(pRange),
    world2TagInit_H(pWorld2TagInit_H),
    nbr_particles(pNbrParticles),
	std_pose(pStdPose),
	std_R(pStdR),
	std_T(pStdT),
    indexMaxLikelihood(0),
    iterated(0)
{
	//init standard librairy seed initiation
	srand((unsigned int) time(0));

	createParticles();
}

// Generate the particle with uniform random value in the range
void ParticleFilter::createParticles(){
	// Fill the array with random value from -1 to 1
	this->particles = Eigen::MatrixXd::Zero(NBR_PARAMETERS, nbr_particles);
	//this->particles.topRows(3) = 1.5 * Eigen::MatrixXd::Random(3, nbr_particles);
	this->particles.topRows(3) = 1.5 * Eigen::MatrixXd::Zero(3, nbr_particles);
	// Set inital position to all particles
	Eigen::Vector3d init_position = world2TagInit_H.col(3).topRows(3);
	ROS_INFO_STREAM(std::endl << "init_position: " << std::endl << init_position);
	for(int i = 0; i < nbr_particles; ++i){
		this->particles.col(i).topRows(3) += init_position;
	}

	// Set each parameter to the correction range
	// for -20 to 20, we multiply by 20 the -1 to 1 random number to get a -20 to 20
	// TODO: Add an actual range instead of this hack
	//for(int i = 0; i < NBR_PARAMETERS; ++i){
	//	particles.row(i) *= range(i);
	//}
	//ROS_INFO_STREAM(std::endl << "Particles initiation value: " << std::endl << particles);

	// Create likelihood vector
	ll = (1.0 / nbr_particles * Eigen::VectorXd::Ones(nbr_particles)).array().log();

	ROS_INFO_STREAM(std::endl << "InitX: " << std::endl << particles);

}

double nrand(){
	double mu = 0.0;
	double sigma = 0.5;
	double r1 = (std::rand() + 1.0)/(RAND_MAX + 1.0); // gives equal distribution in (0, 1]
    double r2 = (std::rand() + 1.0)/(RAND_MAX + 1.0);
    return mu + sigma * std::sqrt(-2*std::log(r1))*std::cos(2*M_PI*r2);
}

void ParticleFilter::updateParticle(){
	// Apply the forces
	particles = forces * particles;
	std_T = 0.03;
	std_R = 0.25;
	ROS_INFO_STREAM(std::endl << "std_T: " << std::endl << std_T);
	ROS_INFO_STREAM(std::endl << "std_R: " << std::endl << std_R);

	// Add random value on the Translation parameter (the first 3 parameters)
	for(int i = 0; i < nbr_particles; ++i){
		particles(0, i) += std_T;// * nrand();
		particles(1, i) += std_T;// * nrand();
		particles(2, i) += std_T;// * nrand();
		particles(3, i) += std_R;// * nrand();
	}
	// Fourth row
	//particles.row(3) += std_R * Eigen::MatrixXd::Random(1, nbr_particles);

	// Update rows 5 to 7
	//particles.bottomRows(4).topRows(3) += std_T / 5.0 * Eigen::MatrixXd::Random(3, nbr_particles);
	particles.bottomRows(4).topRows(3) += std_T / 5.0 * Eigen::MatrixXd::Ones(3, nbr_particles);
	// 8th row
	//particles.row(7) += std_R / 5.0 * Eigen::MatrixXd::Random(1, nbr_particles);
	particles.row(7) += std_R / 5.0 * Eigen::MatrixXd::Ones(1, nbr_particles);
	ROS_INFO_STREAM(std::endl << "UpdateX: " << std::endl << particles);

}

void ParticleFilter::calcLogLikelihood(const  std::list<tagHandle_t> &tags){
	//double A = -log(sqrt(2* M_PI) * std_pose);
	//double B = -0.5 / (std_pose * std_pose);
	std::list<tagHandle_t>::const_iterator it;

	double A = -log(sqrt(2* M_PI) * std_pose);
	double B = -0.5 / (std_pose * std_pose);

	for(int k = 0; k < nbr_particles; ++k){

		bool inRange = true;
		for(int p = 0; p < 4; ++p){
			if(abs(particles(p, k)) > range(p)){
				inRange = false;
			}
		}

		// If 4 first parameters are in range
		if(inRange){
			double yaw = particles(3, k);
			//world2Cube_guess.block(0, 0, 3, 3) = Eigen::AngleAxisd(-0 * M_PI/ 180.0,
			//                                                       Eigen::Vector3d::UnitZ());
			Eigen::Matrix4d world2Cube_guess = Eigen::Matrix4d::Identity(4, 4);
			world2Cube_guess.block(0, 0, 3, 3) = Eigen::AngleAxisd(-yaw * M_PI/ 180.0,
			                                                       Eigen::Vector3d::UnitZ()).matrix();
			world2Cube_guess.col(3).topRows(3) = particles.col(k).topRows(3);
			//world2Cube_guess.col(3).topRows(3) <<  -0.0352,   -0.6409,  -0.0170;
			//world2Cube_R_guess = Eigen::AngleAxisd(-yaw * M_PI/ 180.0, Eigen::Vector3d::UnitZ());
			//const Eigen::Vector3d T_guess = particles.col(k).topRows(3);

			for(it = tags.begin(); it != tags.end(); ++it){
				ROS_INFO_STREAM(std::endl <<"x: "<<std::endl << world2Cube_guess);
				ROS_INFO_STREAM(std::endl <<"x * it->ref.cube2Cam_H: "<<std::endl << world2Cube_guess * it->ref.cube2Cam_H);
				ROS_INFO_STREAM(std::endl <<"it->ref.posTag_W: "<<std::endl <<it->ref.posTag_W);
				ROS_INFO_STREAM(std::endl <<"gros: "<<std::endl <<
				                (world2Cube_guess * it->ref.cube2Cam_H).inverse()*it->ref.posTag_W);
				//Part = PerformCameraProjection(ref.P,(X*ref.HCubeToCam)\ref.PosTagW);
				Eigen::MatrixXd part;
				bool inFrontCam = performCameraProjection(it->ref.proj,
				                                          (world2Cube_guess * it->ref.cube2Cam_H).inverse()*it->ref.posTag_W,
				                                          part);
				//ROS_INFO_STREAM("inFrontCam: "<< std::endl << inFrontCam);
				if(inFrontCam){
					for(int i = 0; i < 4; ++i){
						Eigen::Vector2d error = part.col(i) - it->ref.corners[i];
						double D = error.transpose() * error;

						ll(k) += A + B * D;
					}
				}
				else{
					ll(k) = -std::numeric_limits<double>::max();// = -inf
				}
			}

		}
		else{
			ll(k) = -std::numeric_limits<double>::max();// = -inf
		}
	}
	iterated++;

	ROS_INFO_STREAM(std::endl << "firstL_log: " << std::endl << ll);
}

/**
  * @brief Perform a camera projection on a set of point
  * @param projectionMatrix
  * @param dataPoints
  * @param returnedPoints OUTPUTED points
  * @return true if all points are in front of camera
  */
 bool ParticleFilter::performCameraProjection(Eigen::MatrixXd projectionMatrix, Eigen::MatrixXd dataPoints, Eigen::MatrixXd &returnedPoints){
        Eigen::MatrixXd projectedGPB = projectionMatrix * dataPoints;
		unsigned int nbrPoints = projectedGPB.cols();
		Eigen::MatrixXd imageGPB(2, nbrPoints);
        imageGPB.row(0) = projectedGPB.row(0).cwiseQuotient(projectedGPB.row(2)); // px = v.x / v.z
        imageGPB.row(1) = projectedGPB.row(1).cwiseQuotient(projectedGPB.row(2)); // py = v.y / v.z
		bool inRange = true;
		// TODO use std
		for(int i = 0; i < nbrPoints; ++i){
			if(dataPoints(2,i) <= 0)
				inRange = false;
		}
		returnedPoints = imageGPB;
		return inRange;
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

	//ROS_INFO_STREAM(std::endl << "ll" << std::endl << ll);


	ll = ll - ll.maxCoeff() * Eigen::VectorXd::Ones(nbr_particles);
	Eigen::VectorXd l = ll.array().exp();
	Eigen::VectorXd Q = l / l.sum();
	Eigen::VectorXd R = Q;
	// Currently no equivalant to cumsum in Eigen
	for(int i = 1; i < nbr_particles; ++i)
		R(i) += R(i - 1);

	//Eigen::VectorXd T = Eigen::VectorXd::Random(nbr_particles);
	Eigen::VectorXd T(3); T << 0.2, 0.6, 0.8;

	Eigen::VectorXd ignore(nbr_particles), I(nbr_particles);
	histc(T, R, ignore, I);

	// Resampling, update particles with the
	Eigen::MatrixXd Y = particles;
	for(int i = 0; i < nbr_particles; ++i){
		Y.col(i) = particles.col(I(i)+1);
	}
	particles = Y;
	ll = (1.0 / nbr_particles * Eigen::VectorXd::Ones(nbr_particles)).array().log();


	ROS_INFO_STREAM(std::endl << "sampleX: " << std::endl << particles);


	/*Eigen::VectorXd w(nbr_particles);
	w = ll.array().exp();
    double Wnorm = w.sum();
    w = w / Wnorm;

    double nEff = 1.0/w.array().square().sum();

	if (nEff < 0.5 * nbr_particles){

		Eigen::VectorXd Q = w;
		// Currently no equivalent to cumsum in Eigen
		for(int i = 1; i < nbr_particles; ++i)
			Q(i) += Q(i - 1);

		Eigen::VectorXd T = Eigen::VectorXd::Random(nbr_particles);

		Eigen::VectorXd ignore(nbr_particles), I(nbr_particles);
		histc(T, Q, ignore, I);

		// Resampling, update particles with the
		Eigen::MatrixXd Y = particles;
		for(int i = 0; i < nbr_particles; ++i){
			Y.col(i) = particles.col(I(i)+1);
		}
		particles = Y;
	}*/

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

geometry_msgs::PoseStamped  ParticleFilter::getBestLikelihoodMsg(tagRef_t ref){
	// TODO init at resize this array
	geometry_msgs::PoseStamped m;
	double angle = 0;

	Eigen::VectorXd w = ll.array().exp();
	ROS_INFO_STREAM(std::endl <<
	                "Min: " << w.minCoeff() << std::endl <<
	                "Max:" << w.maxCoeff());
	ROS_INFO_STREAM("pose" << std_pose);
	w = w / w.sum();
	for(int i = 0; i < nbr_particles; ++i){
		m.pose.position.x += particles(0, i) * w(i);
		m.pose.position.y += particles(1, i) * w(i);
		m.pose.position.z += particles(2, i) * w(i);
		angle += particles(3, i) * w(i);
	}
	Eigen::Quaterniond q;
	q = Eigen::AngleAxisd(-angle * M_PI/ 180.0, Eigen::Vector3d::UnitZ());
	m.pose.orientation.x = q.x();
	m.pose.orientation.y = q.y();
	m.pose.orientation.z = q.z();
	m.pose.orientation.w = q.w();

	ROS_INFO_STREAM("Angle " << angle);

	/*Eigen::Matrix<double, 3, 4> proj; //Projection matrixs
	proj << 382.5693969726562,	0.0,				316.7359733365374, 0.0,
	        0.0,				422.2102966308594,	271.7760648319054, 0.0,
	        0.0,				0.0,				1.0,			   0.0;
	Eigen::Vector3d tmp;
	tmp <<
			m.pose.position.x,
			m.pose.position.y,
			m.pose.position.z;
	Eigen::Matrix4d world2Cube_guess = Eigen::Matrix4d::Identity(4, 4);
	world2Cube_guess.col(3).topRows(3) = tmp;
	Eigen::MatrixXd	imgPx(2,1);
	performCameraProjection(proj,
	                        (world2Cube_guess * ref.cube2Cam_H).inverse()*ref.posTag_W,
	                        imgPx);
	//Eigen::Vector2d	imgPx =  proj * tmp;
	//m.pose.position.x = imgPx(0) / imgPx(2) / 640.0;
	//m.pose.position.y = imgPx(1) / imgPx(2) / 480.0;
	m.pose.position.x = imgPx(0) / 640.0;
	m.pose.position.y = imgPx(1) / 480.0;*/

	return m;
}

void ParticleFilter::printQuaternion(const std::string title, const Eigen::Quaterniond &quat) const{
	 ROS_INFO_STREAM(std::endl << title << std::endl << quat.x() << std::endl  << quat.y() << std::endl  << quat.z() << std::endl  << quat.w());
}

void ParticleFilter::updateParameters(double p, double r, double t, double dt, double dr){
	std_pose = p;
	std_T = t;
	std_R = r;
	std_DT = dt;
	std_DR = dr;
}
