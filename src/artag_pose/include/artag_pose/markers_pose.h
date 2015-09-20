#ifndef MARKERSPOSE_H
#define MARKERSPOSE_H

#include <map>
#include <Eigen/Geometry>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>


class MarkerPose
{
public:
	MarkerPose(){
		position =  Eigen::Vector3d(0, 0, 0);
		quat = Eigen::Quaterniond(0, 0, 0, 0);
	}
	MarkerPose(const geometry_msgs::Pose& pose){
		poseMsgToTF(pose, tf);
		position =  Eigen::Vector3d(pose.position.x,
									pose.position.y,
									pose.position.z);
		quat = Eigen::Quaterniond(pose.orientation.w,
								  pose.orientation.x,
								  pose.orientation.y,
								  pose.orientation.z);
	}

	MarkerPose(const tf::Pose& transformation)
		:tf(transformation){}
/*
	Eigen::Vector3d getEigenPosition(){
		return position;
	}

	Eigen::Quaterniond getEigenQuat(){
		return quat;
	}

	geometry_msgs::Pose getRosPose(){
		return pose;
	}*/

	tf::Pose getTf(){
		return tf;
	}

private:
	Eigen::Vector3d position;
	Eigen::Quaterniond quat;
	tf::Pose tf;

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
