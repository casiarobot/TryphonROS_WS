#include "artag_pose/yaml_config_loader.h"

YamlConfigLoader::YamlConfigLoader(ros::NodeHandle *nh):nodeHandle(nh){
}


void YamlConfigLoader::load(int numMarkers){

	ROS_INFO("Didn't crash!");
	std::string configFileName;

	//nodeHandle->param<std::string>("configFileName", configFileName, "");
	if(!nodeHandle->getParam("configFileName", configFileName)){
		ROS_WARN("Can't load list of markers pose from parameter \"configFileName\"");
		ROS_WARN("You might want to use _useYAML=false if you are using tf for markers pose");
		ros::shutdown();
	}
	std::ifstream ifs(configFileName.c_str());

	YAML::Parser parser(ifs);
	YAML::Node doc;
	parser.GetNextDocument(doc);


	for(YAML::Iterator moduleIt = doc.begin(); moduleIt != doc.end(); ++moduleIt){
		const YAML::Node& module(*moduleIt);
	}
	/*
	std::vector<std::map<std::string, > > markers_pose;
	bool parameter_provided = nodeHandle->getParam("markers_pose", markers_pose);

	if(!parameter_provided){
		ROS_WARN("Can't load list of markers pose from parameter \"markers_pose\"");
		ROS_WARN("You might want to use _useYAML=false if you are using tf for markers pose");
		ros::shutdown();
	}*/
}

MarkersPose YamlConfigLoader::parse(){
	return MarkersPose();
}
