#include "default_config.h"

DefaultConfig::DefaultConfig()
{

}

void DefaultConfig::load(int numMarkers){
	double d = 1.00;
	Eigen::Matrix4d tagA, world2Tag;
	tagA << -d/2,  d/2,  d/2,  -d/2,
	         0,  0,  0,  0,
	         -d/2,  -d/2,  d/2,  d/2,
	         1,  1,  1,  1;
	//Eigen::Vector3d tagPosA;
	//tagPosA << -0.0157, 0.3202, 0.0593;
	// z -> y, x -> x, y-> -z
	//tagPosA << -0.00761898502178, 0.29866744968, 0.0571182126474;
//	Eigen::Vector3d tagPosA(-0.73202305202, // old
//	                        11.6619856546,
//							 0.646767046312);
	Eigen::Vector3d tagPosA(1.5,
	                        17,
							 -2.);
	world2Tag = Eigen::Matrix4d::Identity(4,4);
	world2Tag.col(3).topRows(3) = tagPosA;
	world2Tag = world2Tag * tagA;

	m->insert(5, Eigen::Affine3d(world2Tag));
	// C = > second on wall with 2 tag
	// artag -> here
	// z -> y, x -> x, y-> -z
	Eigen::Matrix3d conversionCam;
	conversionCam <<
	                 1,    0,    0,
					 0,    0,    1,
					 0,   -1,    0;
//	Eigen::Vector3d tagPosC(5.72216486695, // old
//	                        11.6977627528,
//	                        -0.68205660388);
	Eigen::Vector3d tagPosC(8,
	                        16,
	                        -2.);
	world2Tag = Eigen::Matrix4d::Identity(4,4);
	world2Tag.col(3).topRows(3) = tagPosC;
//	world2Tag = world2Tag * tagA;

	m->insert(2, Eigen::Affine3d(world2Tag));

	Eigen::Matrix3d rot90;
	rot90 <<  0,    1,    0,
	         -1,    0,    0,
		      0,    0 ,   1;
	world2Tag = Eigen::Matrix4d::Identity(4,4);
	world2Tag.block(0, 0, 3, 3) = rot90;

	//z->x, x-> -y, y->-
//	Eigen::Vector3d tagPosB(13.0752880484 + 1.0, // old
//	                        -6.25567707187 - 1.0,
//	                        3.78939827759);
	Eigen::Vector3d tagPosB(11.6 + 1.0,
	                        -0.1 - 1.0,
	                        2);
	world2Tag.col(3).topRows(3) = tagPosB;
	world2Tag = world2Tag * tagA;

	m->insert(4, Eigen::Affine3d(world2Tag));
}

// Init the tag's position base on artag
void DefaultConfig::init(std::list<tagHandle_t> &tagsDetected){

	std::list<tagHandle_t>::iterator t;

	for(t = tagsDetected.begin(); t != tagsDetected.end(); ++t){

	}
}


MarkersPosePtr DefaultConfig::parse(){
	return m;
}
