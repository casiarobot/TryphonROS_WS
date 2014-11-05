#include "controls.h"
/*#define DEVICE_I2C      "/dev/i2c-3"
typedef enum {FALSE, TRUE}
BOOLEAN;
typedef unsigned char BYTE;
#define INTERACTION     0
*/


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


Eigen::Vector3d forceglobf, forcecubef, posorig, posglobf, quatorig, quatglobf, velorig, velglobf, avelorig, avelglobf;
bool start=true;


void subState(const state::state state)
{
	//Starting Pose//
	if(start){
        posglobf(0)=state.pos[0];
        posglobf(1)=state.pos[1];
        posglobf(2)=state.pos[2];
        quatglobf(0)=state.quat[0];
        quatglobf(1)=state.quat[1];
        quatglobf(2)=state.quat[2];
        quatglobf(3)=state.quat[3];
        velglobf(0)=state.vel[0];
        velglobf(1)=state.vel[1];
        velglobf(2)=state.vel[2];
        avelglobf(0)=state.angvel[0];
        avelglobf(1)=state.angvel[1];
        avelglobf(2)=state.angvel[2];

        start=false;
    }



	// State estimation //
	/*dsx=state.pos[0];
    dsy=state.pos[1];
    dsz=state.pos[2];
    state.vel[0];
    state.vel[1];
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
    */
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "control");
    ros::NodeHandle node;
    ros::Publisher Controle_node = node.advertise<geometry_msgs::Wrench>("command_control",1);
    ros::Rate loop_rate(10);
    Eigen::Matrix3d Rmatrix;

	//ros::Subscriber subA = node.subscribe("/android/imu", 1, poseCallback);
	ros::Subscriber subS = node.subscribe("state", 1, subState);
    geometry_msgs::Wrench wrenchMsg;
	while (ros::ok())
	{
        	
        	////////////////////////////////////
        	////       Controller           ////
        	////////////////////////////////////

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




