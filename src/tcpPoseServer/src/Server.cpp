// Les threads utilisent std:thread et donc il est nécessaire d'utlisé c++11 rajouter le flags  "-std=c++11" à la compilation

#include "Server.h"

using namespace std;

/**
    Initiation of the client socket
*/

Server::Server(){

    // publisher
    pub = nh.advertise<geometry_msgs::Pose>("/desired_deltapose", 1);
    ROS_INFO("Init pub");
    
    // Initiation server
    bzero((char *) &serverSock, sizeof(serverSock));
    serverSock.sin_family = AF_INET;
    serverSock.sin_addr.s_addr = INADDR_ANY;
    serverSock.sin_port = htons(50000);
    this->socketFileDescriptor = socket(AF_INET, SOCK_STREAM, 0);

    this->checkForError(this->socketFileDescriptor, "ERROR opening socket");
    ROS_INFO("A connection");

    this->loop();
}

//sscanf(p->buffer_in, "l-x:%ld/y:%ld/z:%ld/yaw:%ld/pitch:%ld/roll:%ld\n", &lposx, &lposy, &lposz, &lthetax, &lthetay, &lthetaz);
void Server::loop(){
    ROS_INFO("Star loop");
    while(ros::ok()){

        // opening socket
        this->socketFileDescriptor = socket(AF_INET, SOCK_STREAM, 0);
        this->checkForError(this->socketFileDescriptor, "ERROR opening socket");
        ROS_INFO("Socket open");

        // Binding
        int comErrorCode = bind(this->socketFileDescriptor, (struct sockaddr *) &serverSock, sizeof(serverSock));
        if(comErrorCode != 0){
            this->checkForError(comErrorCode, "ERROR on binding");
            this->disconnect();
            sleep(1);
            continue;
        }
        ROS_INFO("Binding successfull");

       listen(socketFileDescriptor, 5);
       ROS_INFO("Listen successfull");

       socklen_t clientAdressLength = (socklen_t) sizeof(serverSock);
       this->comSocket = accept(this->socketFileDescriptor, (struct sockaddr *) &serverSock, &clientAdressLength);
       this->checkForError(this->comSocket, "ERROR on accept");
       ROS_INFO("star receive loop");

       this->receiveLoop();
       this->disconnect();
    }
}

void Server::receiveLoop(){
    char buff[255];
    long int  lposx, lposy, lposz, lthetax, lthetay, lthetaz;
    size_t size = 255;
    int comErrorCode;
    do{
        comErrorCode = recv(comSocket, buff, size, 0);
        this->checkForError(comErrorCode, "ERROR on receive");
        //printf("Message : %s", buff);
        sscanf(buff, "a-x:%ld/y:%ld/z:%ld/yaw:%ld/pitch:%ld/roll:%ld\n", &lposx, &lposy, &lposz, &lthetax, &lthetay, &lthetaz);

        this->generateAndPublishPoseMsg(lposx, lposy, lposz, lthetax, lthetay, lthetaz);
    }while(comErrorCode >= 0 && ros::ok());
}

void Server::generateAndPublishPoseMsg(long int  posx, long int  posy, long int  posz,
                                       long int  thetax, long int  thetay, long int  thetaz){
    geometry_msgs::Pose msg;
    
	    //Convert from angle to Quaterion (SUPPOSE 4DDL ONLY!!!)
	    Eigen::Quaterniond orientation = Eigen::Quaterniond(Eigen::AngleAxisd(thetaz*3.1416/1800, Eigen::Vector3d::UnitZ()));
	    
    msg.position.x = (double)posx/100;
    msg.position.y = (double)posy/100;
    msg.position.z = (double)posz/100;
    msg.orientation.x = orientation.x();
    msg.orientation.y = orientation.y();
    msg.orientation.z = orientation.z();
    msg.orientation.w = orientation.w();
    pub.publish(msg);
}

void Server::checkForError(int errorCode, const char* errorMessage){
    if (errorCode < 0)
        perror(errorMessage);
}

void Server::disconnect(){
  ROS_INFO("Closing tcp_pose_server...\n");
  
    close(this->comSocket);
    close(this->socketFileDescriptor);
}

Server::~Server(){

    close(this->comSocket);
    close(this->socketFileDescriptor);
}


