#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Wrench.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

#include "std_msgs/String.h"
#include "state/state.h"
#include <sstream>

//#include <fcntl.h>
//#include <time.h>
//#include <unistd.h>
//#include <errno.h>
#include <math.h>

//#include "i2c-dev.h"
//#include "motors.h"
//#include "sensors/motor.h"
//#include "sensors/motorArray.h"
//#include "cube.h"       // Cube geometry, inertia, etc.

#define PI 3.14159265

// note on plain values:
// buttons are either 0 or 1
// button axes go from 0 to -1
// stick axes go from 0 to +/-1

#define PS3_BUTTON_SELECT            0
#define PS3_BUTTON_STICK_LEFT        1
#define PS3_BUTTON_STICK_RIGHT       2
#define PS3_BUTTON_START             3
#define PS3_BUTTON_CROSS_UP          4
#define PS3_BUTTON_CROSS_RIGHT       5
#define PS3_BUTTON_CROSS_DOWN        6
#define PS3_BUTTON_CROSS_LEFT        7
#define PS3_BUTTON_REAR_LEFT_2       8
#define PS3_BUTTON_REAR_RIGHT_2      9
#define PS3_BUTTON_REAR_LEFT_1       10
#define PS3_BUTTON_REAR_RIGHT_1      11
#define PS3_BUTTON_ACTION_TRIANGLE   12
#define PS3_BUTTON_ACTION_CIRCLE     13
#define PS3_BUTTON_ACTION_CROSS      14
#define PS3_BUTTON_ACTION_SQUARE     15
#define PS3_BUTTON_PAIRING           16

#define PS3_AXIS_STICK_LEFT_LEFTWARDS    0
#define PS3_AXIS_STICK_LEFT_UPWARDS      1
#define PS3_AXIS_STICK_RIGHT_LEFTWARDS   2
#define PS3_AXIS_STICK_RIGHT_UPWARDS     3
#define PS3_AXIS_BUTTON_CROSS_UP         4
#define PS3_AXIS_BUTTON_CROSS_RIGHT      5
#define PS3_AXIS_BUTTON_CROSS_DOWN       6
#define PS3_AXIS_BUTTON_CROSS_LEFT       7
#define PS3_AXIS_BUTTON_REAR_LEFT_2      8
#define PS3_AXIS_BUTTON_REAR_RIGHT_2     9
#define PS3_AXIS_BUTTON_REAR_LEFT_1      10
#define PS3_AXIS_BUTTON_REAR_RIGHT_1     11
#define PS3_AXIS_BUTTON_ACTION_TRIANGLE  12
#define PS3_AXIS_BUTTON_ACTION_CIRCLE    13
#define PS3_AXIS_BUTTON_ACTION_CROSS     14
#define PS3_AXIS_BUTTON_ACTION_SQUARE    15
#define PS3_AXIS_ACCELEROMETER_LEFT      16
#define PS3_AXIS_ACCELEROMETER_FORWARD   17
#define PS3_AXIS_ACCELEROMETER_UP        18
#define PS3_AXIS_GYRO_YAW                19


#define DEVICE_I2C      "/dev/i2c-3"
typedef enum {FALSE, TRUE}
BOOLEAN;
typedef unsigned char BYTE;
#define INTERACTION     0


geometry_msgs::Wrench  move_to;
geometry_msgs::Wrench wrenchMsg;


double dsx=0;
double dsy=0;
double dsz[5]={0,0,0,0,0};
double dszwant=2000;
double errorn=0;
double erroro=0;
double errorz=0;
double errorzo=0;
double deriv=0;
double derivo=0;
double derivz=0;
double rz=0;
double rzo=0;
bool start=true;
float kz=0.0015;

float incx=0,incy=0,incz=0;

int bool_input(float a,float b)
{

	if (a && !b)
 		return 1;
	else if (!a && b)
		return -1;
	else if (a && b)
		return 0;
	else
		return 0;
}



void joycallback(const sensor_msgs::Joy::ConstPtr& Joy)
{

float k=.06;
 //this is the safety number

float button_x=0, button_y=0, button_z=0;

button_x =bool_input(Joy->buttons[PS3_BUTTON_CROSS_RIGHT],Joy->buttons[PS3_BUTTON_CROSS_LEFT]);
button_y = bool_input(Joy->buttons[PS3_BUTTON_CROSS_UP],Joy->buttons[PS3_BUTTON_CROSS_DOWN]);
button_z =bool_input(Joy->buttons[PS3_BUTTON_REAR_RIGHT_2],Joy->buttons[PS3_BUTTON_REAR_LEFT_2]);

if((incx<30 && button_x>0)||(incx>-30 && button_x<0)){incx=incx+button_x;}
if((incy<30 && button_y>0)||(incy>-30 && button_y<0)){incy=incy+button_y;}
if(button_z)
 {
dszwant+=button_z*20;
	if (dszwant>2300) {dszwant=2300;}

}

	if (Joy->buttons[PS3_BUTTON_ACTION_CIRCLE])
	{
		incx=0;
		incy=0;
	}

	if (Joy->buttons[PS3_BUTTON_ACTION_CROSS])
	{
		incx=0;
		incy=0;
		dszwant=600;
	}


move_to.force.x=k*incx;
move_to.force.y=k*incy;
move_to.torque.x=0;
move_to.torque.y=0;
move_to.torque.z=0;

//ROS_INFO("x:%f, y:%f, z:%f \n", move_to.force.x,move_to.force.y,move_to.force.z);

}

void subState(const state::state state)
{
        //Updating old value //
        erroro=errorn;
        errorz=errorzo;
	for(int i=0;i<5;i++)
		{dsz[i]=dsz[i+1];}
        derivo=deriv;
	// State estimation //
        dsx=state.pos[0];
        dsy=state.pos[1];
        dsz[4]=state.pos[2];
	errorn=dszwant-(dsz[4]);
        /*pose.orientation.x;
        pose.orientation.y;*/
        rz=state.quat[2];
        //pose.orientation.w;

        // Zero compass//
	errorz=sin(rz*PI/180);
	derivz=(errorz-errorzo)/0.05;
	
        // Z-Axis //
        //errorn=dszwant-dsz;
        deriv=(errorn-erroro)/0.05;
	if(abs(deriv)>450){deriv=derivo;}
        // gain  //
	if(abs(errorn)<300){kz=3*0.0015/4;}
	if(abs(errorn)>400){kz=0.0015;}
}

int main(int argc, char** argv)
{

ros::init(argc, argv, "ps3_z");
ros::NodeHandle n;

geometry_msgs::Wrench ft;

move_to.force.x=0;
move_to.force.y=0;
move_to.force.z=0;
move_to.torque.x=0;
move_to.torque.y=0;
move_to.torque.z=0;
int print=0;
ros::Subscriber  joy_stick = n.subscribe<sensor_msgs::Joy>("joy",1,&joycallback);
ros::Subscriber subS = n.subscribe("state", 1, subState);




ros::Publisher  to_control = n.advertise<geometry_msgs::Wrench>("ps3_control",1);
ros::Rate loop_rate(20);

	while (ros::ok())
	{

                ////////////////////////////////////
                ////       Controller           ////
                ////////////////////////////////////

                /*if(dsx<2000){ wrenchMsg.force.x=0.0015*(dsx-1500);}
                if(dsx>2000){ wrenchMsg.force.x=0;}
                if(dsy<2000){ wrenchMsg.force.y=-0.0015*(dsy-1500);}
                if(dsy>2000){ wrenchMsg.force.y=0;}*/
                wrenchMsg.force.x=0;
                wrenchMsg.force.y=0;
                wrenchMsg.force.z=0.001*(errorn+1.65*deriv);
                wrenchMsg.torque.x=0;
                wrenchMsg.torque.y=0;
                wrenchMsg.torque.z=0.18*(errorz+1.18*derivz);
		//ROS_INFO("x:%f, y:%f, z:%f \n", wrenchMsg.force.x,wrenchMsg.force.y,wrenchMsg.force.z);

		ft.force.x=move_to.force.x+wrenchMsg.force.x;
		ft.force.y=move_to.force.y+wrenchMsg.force.y;
		ft.force.z=move_to.force.z+wrenchMsg.force.z;
		ft.torque.x=move_to.torque.x+wrenchMsg.torque.x;
		ft.torque.y=move_to.torque.y+wrenchMsg.torque.y;
		ft.torque.z=move_to.torque.z+wrenchMsg.torque.z;
		if(print==0){print=0;
		ROS_INFO("x:%f, y:%f, z:%f, rz:%f, dszwant:%f, deriv: %f, derivz: %f ", ft.force.x,ft.force.y,ft.force.z,ft.torque.z,dszwant,deriv,derivz);
  		}
		else {++print;}

		to_control.publish(ft);
  		ros::spinOnce();
  		loop_rate.sleep();
	}

return 0;
}
