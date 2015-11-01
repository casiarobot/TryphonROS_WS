#include "artag_pose/particle_filter.h"

using namespace std;

ParticleFilter::ParticleFilter(
				Eigen::MatrixXd pForces,
				Eigen::VectorXd pRange,
                int pNbrParticles,
                double pStdPose,
                double pStdR,
                double pStdT,
				Eigen::Vector3d pCube2Cam_T,
				Eigen::Quaterniond pCube2Cam_R,
				Eigen::Vector3d pWorld2Tag_T,
				Eigen::Quaterniond pWorld2Tag_R):
    forces(pForces),
    range(pRange),
    nbr_particles(pNbrParticles),
	std_pose(pStdPose),
	std_R(pStdR),
	std_T(pStdT),
    cube2Cam_T(pCube2Cam_T),
    cube2Cam_R(pCube2Cam_R),
    world2Tag_T(pWorld2Tag_T),
    world2Tag_R(pWorld2Tag_R)
{
	//init standard librairy seed initiation
	//srand((unsigned int) time(0));

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
	ROS_INFO_STREAM(std::endl << "Particles initiation value: " << std::endl << particles);

	// Create likelihood vector
	ll = Eigen::VectorXd(nbr_particles);
}

void ParticleFilter::update(const Eigen::Vector3d &translation, const Eigen::Quaterniond &rotation){
	ROS_INFO_STREAM(std::endl << "cam2tag translation: " << std::endl << translation);
	printQuaternion(string("cam2tag rotation: "), rotation);

	updateParticle(translation, rotation);
	ROS_INFO_STREAM(std::endl << "Particles updated value: " << std::endl << particles);
	calcLogLikelihood(translation, rotation);
	resampleParticles();
	ROS_INFO_STREAM(std::endl << "Particle end: " << std::endl << particles);
}

void ParticleFilter::updateParticle(const Eigen::Vector3d &translation, const Eigen::Quaterniond &rotation){
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

void ParticleFilter::calcLogLikelihood(const  Eigen::Vector3d &cam2Tag_T, const Eigen::Quaterniond &cam2Tag_R){
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
			Eigen::Matrix3d world2Cube_R_guess;
			world2Cube_R_guess = Eigen::AngleAxisd(-yaw * M_PI/ 180.0, Eigen::Vector3d::UnitZ());
			const Eigen::Vector3d T_guess = particles.col(k).topRows(3);


			Eigen::Vector3d error =
			        (world2Cube_R_guess * cube2Cam_R.matrix().transpose()).inverse()
			        * (world2Tag_T -T_guess -world2Cube_R_guess * cube2Cam_T)
			        -cam2Tag_T;
			double D2 = error.transpose() * error;

			ll(k) = A + B * D2;


			//ROS_INFO_STREAM(std::endl << "yaw: " << std::endl << yaw);
			//ROS_INFO_STREAM(std::endl << "world2Cube_R_guess: " << std::endl << world2Cube_R_guess);;
			//ROS_INFO_STREAM(std::endl << "cube2Cam_R: " << std::endl << cube2Cam_R.matrix().transpose());
			//ROS_INFO_STREAM(std::endl << "(world2Cube_R_guess * cube2Cam_R).inverse(): " << std::endl << (world2Cube_R_guess * cube2Cam_R.matrix().transpose()).inverse());
			//ROS_INFO_STREAM(std::endl << "ll(k) " << std::endl << ll(k));


		}
		else{
			ll(k) = -std::numeric_limits<double>::max();// = -inf
		}
	}


}

// From https://github.com/libigl/libigl/blob/89a8257d152165218fc514e4e35a5aa56dfeca10/include/igl/histc.cpp
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

	//ROS_INFO_STREAM(std::endl << "llorigi:" << std::endl << ll);
	ll = ll - ll.maxCoeff() * Eigen::VectorXd::Ones(nbr_particles);
	ll = ll.array().exp();
	Eigen::VectorXd Q = ll / ll.sum();
	Eigen::VectorXd R = Q;
	// Currently no equivalant to cumsum in Eigen
	for(int i = 1; i < nbr_particles; ++i)
		R(i) += R(i - 1);

	//Eigen::VectorXd T = Eigen::VectorXd::Random(nbr_particles);
	Eigen::VectorXd T(nbr_particles);
	T <<  0.146637, 0.511162, -0.896122, -0.684386, 0.999987, -0.591343, 0.779911, -0.749063, 0.995598, -0.891885;

	Eigen::VectorXd ignore(nbr_particles), I(nbr_particles);
	histc(T, R, ignore, I);

	// Resampling, update particles with the
	Eigen::MatrixXd Y = particles;
	for(int i = 0; i < nbr_particles; ++i){
		Y.col(i) = particles.col(I(i)+1);
	}
	particles = Y;

	/*ROS_INFO_STREAM(std::endl << "ll:" << std::endl << ll);
	ROS_INFO_STREAM(std::endl << "Q:" << std::endl << Q);
	ROS_INFO_STREAM(std::endl << "R:" << std::endl << R);
	ROS_INFO_STREAM(std::endl << "T:" << std::endl << T);
	ROS_INFO_STREAM(std::endl << "I:" << std::endl << I);
	ROS_INFO_STREAM(std::endl << "Y:" << std::endl << Y);*/

}
void ParticleFilter::printQuaternion(const std::string title, const Eigen::Quaterniond &quat) const{
	 ROS_INFO_STREAM(std::endl << title << std::endl << quat.x() << std::endl  << quat.y() << std::endl  << quat.z() << std::endl  << quat.w());
}
