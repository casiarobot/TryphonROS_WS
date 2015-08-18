#ifndef CONFIGLOADER_H
#define CONFIGLOADER_H

#include "artag_pose/markers_pose.h"

class ConfigLoader
{
protected:
	MarkersPosePtr m;

public:
	ConfigLoader():
		m(new MarkersPose)
	{}
	virtual void load(int numMarkers) = 0;
	virtual MarkersPosePtr parse() = 0;
};

#endif // CONFIGLOADER_H
