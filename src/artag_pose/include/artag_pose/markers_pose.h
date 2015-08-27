#ifndef MARKERSPOSE_H
#define MARKERSPOSE_H

#include <map>
#include <Eigen/Geometry>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>

#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>


class MarkerPose
{
public:
	MarkerPose(){}
	MarkerPose(const geometry_msgs::Pose& pose):
		poseRos(pose){
		tf::poseMsgToTF(pose, poseTf);
		tf::poseMsgToEigen(pose, poseEigen);
	}

	MarkerPose(const tf::Pose& tf)
		:poseTf(tf){
		tf::poseTFToMsg(tf, poseRos);
		tf::poseTFToEigen(tf, poseEigen);

	}

	MarkerPose(const Eigen::Affine3d& eigen)
		:poseEigen(eigen){
		tf::poseEigenToMsg(eigen, poseRos);
		tf::poseEigenToTF(eigen, poseTf);

	}

	Eigen::Affine3d getEigen(){
		return poseEigen;
	}

	geometry_msgs::Pose getRosPose(){
		return poseRos;
	}

	tf::Pose getTf(){
		return poseTf;
	}

private:
	geometry_msgs::Pose poseRos;
	Eigen::Affine3d poseEigen;
	tf::Pose poseTf;

};

class MarkersPose
{
public:
	void insert(const unsigned int & id, const geometry_msgs::Pose & pose){
		map.insert(std::pair<unsigned int, MarkerPose>(id, pose));
	}
	void insert(const unsigned int & id, const tf::Pose & tf){
		map.insert(std::pair<unsigned int, MarkerPose>(id, tf));
	}

	bool isValidMarker(const unsigned int & id){
		return map.find(id) != map.end();
	}

	MarkerPose get(const unsigned int & id){
		return map[id];
	}
	MarkerPose operator[](const unsigned int & id){
		return map[id];
	}

private:
	std::map<unsigned int, MarkerPose> map;
};

typedef boost::shared_ptr<MarkersPose> MarkersPosePtr;

#endif // MARKERSPOSE_H
