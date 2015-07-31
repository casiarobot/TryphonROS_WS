#ifndef MARKERSPOSE_H
#define MARKERSPOSE_H

#include <map>
#include <Eigen/Geometry>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>

class MarkerPose
{
public:
	MarkerPose(){
		position =  Eigen::Vector3d(0, 0, 0);
		quat = Eigen::Quaterniond(0, 0, 0, 0);
	}
	MarkerPose(geometry_msgs::Pose pose){
		tf.translation.x = pose.position.x;
		tf.translation.y = pose.position.y;
		tf.translation.z = pose.position.z;
		tf.rotation.x = pose.orientation.x;
		tf.rotation.y = pose.orientation.y;
		tf.rotation.z = pose.orientation.z;
		position =  Eigen::Vector3d(tf.translation.x,
									tf.translation.y,
									tf.translation.z);
		quat = Eigen::Quaterniond(tf.rotation.w,
								  tf.rotation.x,
								  tf.rotation.y,
								  tf.rotation.z);
	}

	MarkerPose(geometry_msgs::Transform transformation):
		tf(transformation){
	}
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

	geometry_msgs::Transform getTf(){
		return tf;
	}

private:
	Eigen::Vector3d position;
	Eigen::Quaterniond quat;
	geometry_msgs::Transform tf;

};

class MarkersPose
{
public:
	void insert(const unsigned int & id, const geometry_msgs::Pose & pose){
		map.insert(std::pair<unsigned int, MarkerPose>(id, pose));
	}
	void insert(const unsigned int & id, const geometry_msgs::Transform & tf){
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

#endif // MARKERSPOSE_H
