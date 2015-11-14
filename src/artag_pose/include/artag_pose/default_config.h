#ifndef DEFAULTCONFIG_H
#define DEFAULTCONFIG_H

// Standard C/C++ libraries
#include <math.h>
#include <string.h>
#include <vector>

#include <geometry_msgs/Transform.h>

//library for ros
#include <ros/ros.h>
#include "artag_pose/config_loader.h"
#include "artag_pose/particle_filter.h"

class DefaultConfig : public ConfigLoader
{
public:
	DefaultConfig();
	void load(int numMarkers);
	void init(const std::list<tagHandle_t> &tagsDetected);
	MarkersPosePtr parse();
};

#endif // DEFAULTCONFIG_H
