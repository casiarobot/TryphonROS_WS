#ifndef _LEDSFINDER_H_
#define _LEDSFINDER_H_

#include <time.h>
#include <stdlib.h> //calloc
#include <vector>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <netdb.h> 
#include <string>
#include <iostream>
#include <cstdlib> // exit()
#include <fstream>
#include <sstream>
#include <iostream> 
#include <pthread.h>
#include <time.h>

//ROS
#include <ros/ros.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <geometry_msgs/PoseStamped.h>
class Server
{
private:
    // TCP
    struct sockaddr_in serverSock;
    int comSocket;
    int socketFileDescriptor;
   
    ros::NodeHandle nh;
    ros::Publisher pub;
public:
	Server();
	void disconnect();
	void checkForError(int errorCode, const char* errorMessage);

	void loop();
	void receiveLoop();
	void generateAndPublishPoseMsg(long int  posx, long int  posy, long int  posz, long int  thetax, long int  thetay, long int  thetaz);
	~Server();
};

#endif

