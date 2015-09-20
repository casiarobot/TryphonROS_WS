#ifndef TFCONFIGLOADER_H
#define TFCONFIGLOADER_H

// Standard C/C++ libraries
#include <math.h>
#include <string.h>
#include <vector>

#include <geometry_msgs/Transform.h>

//library for ros
#include <ros/ros.h>
#include "artag_pose/config_loader.h"

class TfConfigLoader : public ConfigLoader
{
public:
	TfConfigLoader();
	void load(int numMarkers);
	MarkersPosePtr parse();
};

#endif // TFCONFIGLOADER_H
