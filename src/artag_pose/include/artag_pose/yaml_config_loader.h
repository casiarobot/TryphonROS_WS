#ifndef YAMLCONFIGLOADER_H
#define YAMLCONFIGLOADER_H

// Standard C/C++ libraries
#include <math.h>
#include <string.h>
#include <vector>
#include <fstream>      // std::ifstream

#include <yaml-cpp/yaml.h>

//library for ros
#include <ros/ros.h>
#include "artag_pose/config_loader.h"

class YamlConfigLoader : public ConfigLoader
{
public:
	YamlConfigLoader(){}
	YamlConfigLoader(ros::NodeHandle* nodeHandle);
	void load(int numMarkers);
	MarkersPosePtr parse();
private:
	ros::NodeHandle* nodeHandle;
};

#endif // YAMLCONFIGLOADER_H
