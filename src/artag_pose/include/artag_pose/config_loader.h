#ifndef CONFIGLOADER_H
#define CONFIGLOADER_H

#include "artag_pose/markers_pose.h"

class ConfigLoader
{
protected:
	MarkersPose m;

public:
	ConfigLoader(){}
	virtual void load(int numMarkers) = 0;
	virtual MarkersPose parse() = 0;
};

#endif // CONFIGLOADER_H
