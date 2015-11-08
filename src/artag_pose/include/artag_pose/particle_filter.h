#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include <string>
#include <math.h>
#include <limits.h>
#include <list>

#include "ros/ros.h"
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions>


typedef struct{
	Eigen::Vector3d cube2Cam_T;
	Eigen::Matrix3d cube2Cam_R;
	Eigen::Vector3d world2Tag_T;
	Eigen::Matrix3d world2Tag_R;
}
tagRef_t;

typedef struct {
	Eigen::Vector3d cam2Tag_T;
	Eigen::Matrix3d  cam2Tag_R;
	tagRef_t ref;
}
tagHandle_t;


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
	double std_pose, std_R, std_T, std_DT, std_DR;
	/* Likelihood */
	Eigen::VectorXd ll;

	int indexMaxLikelihood;

	// TODO use a struct
	// Camera and tag reference
	//Eigen::Vector3d cube2Cam_T;
	//Eigen::Quaterniond cube2Cam_R;
	//Eigen::Vector3d world2Tag_T;
	//Eigen::Quaterniond world2Tag_R;

public:
	ParticleFilter(const Eigen::MatrixXd& forces,
	               const Eigen::VectorXd& range,
	               int nbr_particles,
	               double std_pose,
	               double std_R,
	               double std_T);
	void createParticles();
	void update();
	void updateParticle();
	void calcLogLikelihood(const  std::list<tagHandle_t> &tags,
	                       const bool reculsive_flag);
	void resampleParticles();

	geometry_msgs::PoseArray getParticleMsg();
	geometry_msgs::PoseStamped getBestLikelihoodMsg();
	void updateParameters(double p, double r, double t, double dt, double dr);
private:

	void printQuaternion(const std::string title, const Eigen::Quaterniond &quat) const;

	const static int NBR_PARAMETERS = 8;

};


#endif // PARTICLE_FILTER_H
