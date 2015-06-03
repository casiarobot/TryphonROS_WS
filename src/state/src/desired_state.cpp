#include "ros/ros.h"
#include "geometry_msgs/Pose.h"

#include <cstdlib>
#include <cstring>
#include <cstdio>

#include <iostream>

//using namespace std;


int main ( int argc, char **argv)
{

	ros::init(argc,argv,"desiredpose");
	ros::NodeHandle node;
	ros::Publisher desired_state = node.advertise<geometry_msgs::Pose> ("dpose",20);
	ros::Rate loop_rate(20);

	char pose_change[3],y_n; 
        float pose_new;
	geometry_msgs::Pose pose_command;

	if (argc<7) //make sure enough arguments are input
		{
			ROS_INFO("Not enough arguments given. Please Re-run Node and give all 7 positions"); 
			return 0;
		}

	pose_command.position.x=atof(argv[1]); //assign argument values to Pose
	pose_command.position.y=atof(argv[2]);
	pose_command.position.z=atof(argv[3]);
	pose_command.orientation.x=atof(argv[4]);
	pose_command.orientation.y=atof(argv[5]);
	pose_command.orientation.z=atof(argv[6]);
	pose_command.orientation.w=atof(argv[7]);

//ensure the input values are the intended values
ROS_INFO("These are the values ---> x:%f,y:%f,z:%f,rx:%f,ry:%f,rz:%f,rw:%f \n", pose_command.position.x, pose_command.position.y, pose_command.position.z,pose_command.orientation.x, pose_command.orientation.y, pose_command.orientation.z, pose_command.orientation.w);
	std::cout << "Do you accept: y/n";

 	std::cin>>y_n;

if (y_n != 'y') //if anything other than y is entered, the Node will close
    {
    	std::cout << "Not correct values! Restart Node";
    	return 0;
    }

desired_state.publish(pose_command);


	while (ros::ok())
	{
		std::cout << "\nWhat Pose would you like to change? x, y, z, rx, ry, rz, rw, original:  ";
		std::cin>>pose_change;
		
		if(strncmp(pose_change,"origial",4))
		{
		std::cout << "\nWhat Value?";
		std::cin>>pose_new;
		}

		if (!strncmp(pose_change,"x",1))
		{
			pose_command.position.x=pose_new;
		}
		else if (!strncmp(pose_change,"y",1))
		{
			pose_command.position.y=pose_new;/* code */
		}
		else if (!strncmp(pose_change,"z",1))
		{
			pose_command.position.z=pose_new;/* code */
		}
		else if (!strncmp(pose_change,"rx",2))
		{
			pose_command.orientation.x=pose_new;/* code */
		}
		else if (!strncmp(pose_change,"ry",2))
		{
			pose_command.orientation.y=pose_new;/* code */
		}
		else if (!strncmp(pose_change,"rz",2))
		{
			pose_command.orientation.z=pose_new;/* code */
		}
		else if (!strncmp(pose_change,"rw",2))
		{
			pose_command.orientation.w=pose_new;/* code */
		}
		else if (!strncmp(pose_change,"original",4))
		{
			pose_command.position.x=atof(argv[1]); //assign argument values back to Pose
			pose_command.position.y=atof(argv[2]);
			pose_command.position.z=atof(argv[3]);
			pose_command.orientation.x=atof(argv[4]);
			pose_command.orientation.y=atof(argv[5]);
			pose_command.orientation.z=atof(argv[6]);
			pose_command.orientation.w=atof(argv[7]);
		}
		else
		{
			std::cout<< "\nThis position does not exist";
		}
		
		ROS_INFO("These are the values ---> x:%f,y:%f,z:%f,rx:%f,ry:%f,rz:%f,rw:%f \n", pose_command.position.x, pose_command.position.y, pose_command.position.z,pose_command.orientation.x, pose_command.orientation.y, pose_command.orientation.z, pose_command.orientation.w);
		
		std::cout << "Do you accept: y/n : ";
		y_n='n';
 		std::cin>>y_n;

	if (y_n != 'y') //if anything other than y is entered, the Node will close
   	 {
    		std::cout << "Not correct values! Restart Node";
    		return 0;
   	 }
		desired_state.publish(pose_command);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}



