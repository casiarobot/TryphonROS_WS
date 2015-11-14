#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include <string>
#include <math.h>
#include <limits.h>
#include <list>

#include "ros/ros.h"
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>

// For normal distribution
#include <cmath>

#include <Eigen/Geometry>
#include <Eigen/LU>
#include <unsupported/Eigen/MatrixFunctions>


typedef struct{
	Eigen::Matrix4d posTag_W; // world to Tag

	Eigen::Matrix4d cube2Cam_H;
/*	Eigen::Vector3d cube2Cam_T;
	Eigen::Matrix3d cube2Cam_R;
	Eigen::Vector3d world2Tag_T;
	Eigen::Matrix3d world2Tag_R;*/
	Eigen::Matrix<double, 3, 4> proj; //Projection matrixs
	Eigen::Vector2d corners[4]; // Corner order: sw, se, ne, nw
	Eigen::Vector3d world2Tag_T_corners[4];\

}
tagRef_t;

typedef struct {
	geometry_msgs::Point cam2Tag_T;
//	Eigen::Matrix4d cam2Tag_H;
//	Eigen::Vector3d cam2Tag_T;
//	Eigen::Matrix3d cam2Tag_R;
	int id;
	tagRef_t ref;
}
tagHandle_t;


class ParticleFilter
{
	int iterated;
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


	Eigen::Matrix4d world2TagInit_H;

public:
	ParticleFilter(const Eigen::MatrixXd& forces,
	               const Eigen::VectorXd& range,
	               const Eigen::Matrix4d& world2TagInit_H,
	               int nbr_particles,
	               double std_pose,
	               double std_R,
	               double std_T);
	void createParticles();
	void update();
	void updateParticle();
	void calcLogLikelihood(const  std::list<tagHandle_t> &tags);
	void resampleParticles();
	double getVariance();

	geometry_msgs::PoseArray getParticleMsg(const Eigen::Vector3d &offset);
	geometry_msgs::PoseStamped getBestLikelihoodMsg(const Eigen::Vector3d &offset);
	void updateParameters(double p, double r, double t, double dt, double dr);
private:
	bool performCameraProjection(Eigen::MatrixXd projectionMatrix,
	                             Eigen::MatrixXd dataPoints,
	                             Eigen::MatrixXd &returnedPoints);
	void printQuaternion(const std::string title, const Eigen::Quaterniond &quat) const;

	const static int NBR_PARAMETERS = 8;

};


#endif // PARTICLE_FILTER_H
