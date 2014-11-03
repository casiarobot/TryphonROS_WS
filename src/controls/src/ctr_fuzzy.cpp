#include "controls.h"



/*#define DEVICE_I2C      "/dev/i2c-3"
typedef enum {FALSE, TRUE}
BOOLEAN;
typedef unsigned char BYTE;
#define INTERACTION     0
*/


typedef struct
{
    double Fx;                              // The Force in x-Direction
    double Fy;                              // The Force in y-Direction
    double Fz;                              // The Force in z-Direction
    double Tx;                              // The Torque around x-Direction
    double Ty;                              // The Torque around y-Direction
    double Tz;                              // The Torque around z-Direction
}
t_forces;

double dsz=0;
double dsx=0;
double dsy=0;
double dszwant=1000;
double errorn=0;
double erroro=0;
double errorz=0;
double deriv=0;
double rz=0;
double rzo=0;
double rz0=0;
int rzpos=1; // 0 between -360 and 0; 1-> 0-360; 2->360-720
bool start=true;

void subState(const state::state state)
{
	//Updating old value //
	erroro=errorn;
	rzo=rz;

	// State estimation //
	dsx=state.pos[0];
    	dsy=state.pos[1];
    	dsz=state.pos[2];
    	/*pose.orientation.x;
    	pose.orientation.y;*/
    	rz=state.quat[2];
    	//pose.orientation.w;

	// Zero compass//
	if(start){
    	rz0=rz;
    	start=false;}

    	// Z-Axis //
	erroro=errorn;
    	errorn=dszwant-dsz;
    	deriv=(errorn-erroro)/0.05;

	// RZ  //
	//if(rzy
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "control");
    ros::NodeHandle node;
    ros::Publisher Controle_node = node.advertise<geometry_msgs::Wrench>("command_control",1);
    ros::Rate loop_rate(20);

	//ros::Subscriber subA = node.subscribe("/android/imu", 1, poseCallback);
	ros::Subscriber subS = node.subscribe("state", 1, subState);
	while (ros::ok())
	{
        	geometry_msgs::Wrench wrenchMsg;
        	////////////////////////////////////
        	////       Controller           ////
        	////////////////////////////////////

        	/*if(dsx<2000){ wrenchMsg.force.x=0.0015*(dsx-1500);}
        	if(dsx>2000){ wrenchMsg.force.x=0;}
       		if(dsy<2000){ wrenchMsg.force.y=-0.0015*(dsy-1500);}
    		if(dsy>2000){ wrenchMsg.force.y=0;}*/
		wrenchMsg.force.x=0;
		wrenchMsg.force.y=0;
		wrenchMsg.force.z=0.0012*(errorn+1.2*deriv);
      	  	wrenchMsg.torque.x=0;
        	wrenchMsg.torque.y=0;
    		wrenchMsg.torque.z=-0.012*errorz;

		ROS_INFO("fx: %f, fy: %f, fz: %f,Tx: %f, Ty: %f, Tz: %f",wrenchMsg.force.x,wrenchMsg.force.y,wrenchMsg.force.z,wrenchMsg.torque.x,wrenchMsg.torque.y,wrenchMsg.torque.z);
       		 /////////////////////////////////
        	ROS_INFO("error z",errorz);
        	Controle_node.publish(wrenchMsg);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}




