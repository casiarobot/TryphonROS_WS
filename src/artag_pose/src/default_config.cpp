#include "default_config.h"

DefaultConfig::DefaultConfig()
{

}

void DefaultConfig::load(int numMarkers){
	Eigen::Matrix4d world2Tag;
	world2Tag = Eigen::Matrix4d::Identity(4,4);

	m->insert(5, Eigen::Affine3d(world2Tag));
	m->insert(2, Eigen::Affine3d(world2Tag));
	m->insert(4, Eigen::Affine3d(world2Tag));
}

// Init the tag's position base on the first valid artag msg
void DefaultConfig::init(const std::list<tagHandle_t> &tagsDetected){
	double d = 1.00;
	Eigen::Matrix4d tag4Corner, world2Tag;
	tag4Corner <<
	         -d/2,  d/2,  d/2,  -d/2,
	         0,       0,      0,    0,
	         -d/2,  -d/2,  d/2,  d/2,
	         1,        1,    1,    1;
	Eigen::Matrix3d rot90;
	rot90 <<  0,    1,    0,
	         -1,    0,    0,
		      0,    0 ,   1;

	std::list<tagHandle_t>::const_iterator t;
	for(t = tagsDetected.begin(); t != tagsDetected.end(); ++t){
		world2Tag = Eigen::Matrix4d::Identity(4,4);
		Eigen::Vector3d tagPos;
		switch(t->id){
			case 5:
			case 2:// z -> y, x -> x, y-> -z
				tagPos = Eigen::Vector3d(t->cam2Tag_T.x,
										 t->cam2Tag_T.z,
				                        -t->cam2Tag_T.y);
				break;
			case 4://z->x, x-> -y, y->-z
				tagPos = Eigen::Vector3d(t->cam2Tag_T.z + 1.0,
										-t->cam2Tag_T.x - 1.0,
				                        -t->cam2Tag_T.y);
				world2Tag.block(0, 0, 3, 3) = rot90;
				break;
			default:
				continue;
		}
		world2Tag.col(3).topRows(3) = tagPos;
		world2Tag = world2Tag * tag4Corner;
		m->get(t->id).setEigen(Eigen::Affine3d(world2Tag));
	}
}


MarkersPosePtr DefaultConfig::parse(){
	return m;
}
