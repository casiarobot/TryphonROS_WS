#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/package.h>
#include <std_msgs/Float64.h>

// Standard C/C++ libraries
#include <stdio.h>
#include <stdlib.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <ctime>
#include <iostream>
#include <fstream>
#include <cmath>
#include <string>
#include <vector>
#include <iterator>
#include <sstream>

#include <boost/lexical_cast.hpp>



int main(int argc, char **argv){
	ROS_INFO("in main");
	std::string input;
	std::string inputLine;
	std::ifstream csvFile;
    int inputCount = 0;
    int posnCount = 0;
    double inputValue;

    ros::init(argc,argv,"csv_to_bagARTAG1");
    ros::NodeHandle n;
    ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseStamped>("/192_168_10_243/artags1/artag/ar_pose_marker",1000);
    ros::Time startTime = ros::Time::now();
    ros::Rate loop_rate(10);

	// csvFile.open("nominal-maneuver-poseStamped.csv", std::ios::in);
	csvFile.open("/home/tryphon/ROSTryphon_groundstation/src/vicon_filter/path-maneuver-60Hz-fullMonteCarloARTAG1.csv", std::fstream::in | std::fstream::app);

	//csvFile.open("nominal-pose.txt", std::fstream::in | std::fstream::app);
	//csvFile.open(ros::package::getPath("vicon_filter")+"/nominal-maneuver-poseStamped.txt", std::fstream::in | std::fstream::app);
	if (csvFile.is_open()){
		ROS_INFO("Successfully opened file");
		
		//while (ros::ok()){

		while(getline(csvFile,inputLine)){
			//ROS_INFO("stringLIne: %s",inputLine.c_str());
			std::istringstream inputLineStream(inputLine.c_str());
			

			geometry_msgs::PoseStamped pose_msg;

			while(getline(inputLineStream,input,',')){
				ROS_INFO("stringValue: %s",input.c_str());
				if (inputCount > 9){
					//inputValue = boost::lexical_cast<double>(input.c_str());						
					if (posnCount == 0){
						inputValue = boost::lexical_cast<int>(input.c_str());
						ROS_INFO("header.seq: %f",inputValue);
						pose_msg.header.seq = inputValue;
					}
					else if (posnCount == 1){
						ROS_INFO("header.frame_id: %s",input.c_str());
						pose_msg.header.frame_id = input.c_str();
					}
					else if (posnCount == 2){
						inputValue = boost::lexical_cast<long double>(input.c_str());
						ROS_INFO("inputTime: %f",inputValue);
						ros::Duration inputDuration(inputValue);
						pose_msg.header.stamp = ros::Time::now();//startTime + inputDuration;
					}
					else if (posnCount == 3){
						inputValue = boost::lexical_cast<long double>(input.c_str());
						ROS_INFO("position.x: %f",inputValue);
						pose_msg.pose.position.x = inputValue;
					}
					else if (posnCount == 4){
						inputValue = boost::lexical_cast<long double>(input.c_str());
						ROS_INFO("position.y: %f",inputValue);
						pose_msg.pose.position.y = inputValue;
					}
					else if (posnCount == 5){
						inputValue = boost::lexical_cast<long double>(input.c_str());
						ROS_INFO("position.z: %f",inputValue);
						pose_msg.pose.position.z = inputValue;
					}
					else if (posnCount == 6){
						inputValue = boost::lexical_cast<long double>(input.c_str());
						ROS_INFO("quat.w: %f",inputValue);
						pose_msg.pose.orientation.w = inputValue;
					}
					else if (posnCount == 7){
						inputValue = boost::lexical_cast<long double>(input.c_str());
						ROS_INFO("quat.x: %f",inputValue);
						pose_msg.pose.orientation.x = inputValue;
					}
					else if (posnCount == 8){
						inputValue = boost::lexical_cast<long double>(input.c_str());
						ROS_INFO("quat.y: %f",inputValue);
						pose_msg.pose.orientation.y = inputValue;
					}
					else if (posnCount == 9){
						inputValue = boost::lexical_cast<long double>(input.c_str());
						ROS_INFO("quat.z: %f",inputValue);
						pose_msg.pose.orientation.z = inputValue;
					}
					else {ROS_ERROR("posnCount error");}

				}
				inputCount ++;
				posnCount ++;
				if (posnCount > 9){posnCount = 0;}

			}
			//publish
			if(ros::ok()) {
				pose_pub.publish(pose_msg);
				loop_rate.sleep();
			}
			else {ROS_ERROR("ROS NOT OKAY");}				
			
		}

		//}
		csvFile.close();
		ROS_INFO("inputCount: %d", inputCount);

	}
	else ROS_INFO("Could not open file");


	return 0;
}