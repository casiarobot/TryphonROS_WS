#include "Server.h"
#include <signal.h>

Server* server;

void mySigintHandler(int sig)
{
   // Do some custom action.
   // For example, publish a stop message to some other nodes.
   server->disconnect();
   // All the default sigint handler does is call shutdown()
   ros::shutdown();
}

int main(int argc, char* argv[]){

	ROS_INFO("Start tcp_pose_server...\n");
	//ros::init(argc, argv,  std::string("image_publisher")+std::string(argv[1]));
	ros::init(argc, argv,  "tcp_pose_server");
	
    
    signal(SIGINT, mySigintHandler);
    
    server = new Server();
  	ros::spin();
	//client.foo();
	return 0;
}
