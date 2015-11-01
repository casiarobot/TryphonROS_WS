#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include <string>
#include <math.h>
#include <limits.h>

#include "ros/ros.h"
#include <Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions>

class ParticleFilter
{
	/* Matrix representing the model cinetic property */
	Eigen::MatrixXd forces;
	/* Max value of the 4 first parameters*/
	Eigen::VectorXd range;
	/* Collums are particles, rows are parameter */
	/* Parameters are x, y, z, yaw, dx, dy, dz, dyaw*/
	Eigen::MatrixXd particles;
	int nbr_particles;
	/* Standard deviation */
	double std_pose, std_R, std_T;
	/* Likelihood */
	Eigen::VectorXd ll;

	// TODO use a struct
	// Camera and tag reference
	Eigen::Vector3d cube2Cam_T;
	Eigen::Quaterniond cube2Cam_R;
	Eigen::Vector3d world2Tag_T;
	Eigen::Quaterniond world2Tag_R;

public:
	ParticleFilter(Eigen::MatrixXd forces,
	               Eigen::VectorXd range,
	               int nbr_particles,
	               double std_pose,
	               double std_R,
	               double std_T,
	               Eigen::Vector3d pCube2Cam_T,
				   Eigen::Quaterniond pCube2Cam_R,
	               Eigen::Vector3d pWorld2Tag_T,
	               Eigen::Quaterniond pWorld2Tag_R);
	void createParticles();
	void update(const  Eigen::Vector3d &translation,
	            const Eigen::Quaterniond &rotation);
private:
	void updateParticle(const  Eigen::Vector3d &translation,
	                    const Eigen::Quaterniond &rotation);
	void calcLogLikelihood(const  Eigen::Vector3d &translation,
	                       const Eigen::Quaterniond &rotation);
	void resampleParticles();

	void printQuaternion(const std::string title, const Eigen::Quaterniond &quat) const;

	const static int NBR_PARAMETERS = 8;

};

#endif // PARTICLE_FILTER_H
