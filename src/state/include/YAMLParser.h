#ifndef YAML_PARSER_H
#define YAML_PARSER_H

#include <yaml-cpp/yaml.h>
#include <fstream>
#include <geometry_msgs/Pose.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace yaml{

/// Reads a Point message from a YAML node
void operator >> (const YAML::Node& node,  Eigen::Vector3d &p)
{
  node[0] >> p[0];
  node[1] >> p[1];
  node[2] >> p[2];
}

/// Reads a Quaternion message from a YAML node
void operator >> (const YAML::Node& node,  Eigen::Quaterniond &r)
{
  double x,y,z,w;

  node[0] >> x;
  node[1] >> y;
  node[2] >> z;
  node[3] >> w;

  Eigen::Quaterniond q(x,y,z,w);
  r=q;
}


inline void loadPose(std::string filename, Eigen::Vector3d &pos, Eigen::Quaterniond &rot)
{
  std::ifstream ifs(filename.c_str());

  if(!ifs.good())
  {
    ROS_FATAL_STREAM("YAMLParser: Couldn't open "<<filename<<" to get pose information");
    ros::shutdown();
  }

  try {
    YAML::Parser parser(ifs);
    if (!parser)
    {
      ROS_FATAL("YAMLParser: Unable to create YAML parser for loading pose");
      ros::shutdown();
    }
    YAML::Node doc;
    parser.GetNextDocument(doc);

    doc["position"] >> pos;
    doc["orientation"] >> rot;
  }
  catch (YAML::Exception& e) {
    ROS_FATAL("YAMLParser: Exception parsing YAML when loading pose:\n%s", e.what());
    ros::shutdown();
  }
}

inline void loadPosition(std::string filename, Eigen::Vector3d &pos)
{
  std::ifstream ifs(filename.c_str());

  if(!ifs.good())
  {
    ROS_FATAL_STREAM("YAMLParser: Couldn't open "<<filename<<" to get pose information");
    ros::shutdown();
  }

  try {
    YAML::Parser parser(ifs);
    if (!parser)
    {
      ROS_FATAL("YAMLParser: Unable to create YAML parser for loading pose");
      ros::shutdown();
    }
    YAML::Node doc;
    parser.GetNextDocument(doc);

    doc["position"] >> pos;
  }
  catch (YAML::Exception& e) {
    ROS_FATAL("YAMLParser: Exception parsing YAML when loading pose:\n%s", e.what());
    ros::shutdown();
  }
}

inline void loadOrientation(std::string filename, Eigen::Quaterniond &rot)
{
  std::ifstream ifs(filename.c_str());

  if(!ifs.good())
  {
    ROS_FATAL_STREAM("YAMLParser: Couldn't open "<<filename<<" to get pose information");
    ros::shutdown();
  }

  try {
    YAML::Parser parser(ifs);
    if (!parser)
    {
      ROS_FATAL("YAMLParser: Unable to create YAML parser for loading pose");
      ros::shutdown();
    }
    YAML::Node doc;
    parser.GetNextDocument(doc);

    doc["orientation"] >> rot;
  }
  catch (YAML::Exception& e) {
    ROS_FATAL("YAMLParser: Exception parsing YAML when loading pose:\n%s", e.what());
    ros::shutdown();
  }
}

} // end namespace yaml

#endif
