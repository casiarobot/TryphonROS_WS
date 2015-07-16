#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Wrench.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensors/compass.h>
#include <controls/Commands.h>
#include <std_msgs/Bool.h>

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


geometry_msgs::Pose  move_to;
geometry_msgs::Pose  default_pose;
controls::Commands com;
geometry_msgs::Wrench manual;
int mode =1;
std_msgs::Bool magnet_on;
double temp_magn=0;
int yaw_ref=0;
double err_yaw_old=0;
double vel_yaw_old=0;
bool yaw_start=true;
double ctrl_yaw=0;
double t_yaw=0;

ros::Publisher Desired_pose_node;
ros::Publisher Control_node;
ros::Publisher Magnet;

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

void sendit()
{
	com.deltaPose.position.x=move_to.position.x+default_pose.position.x;
	com.deltaPose.position.y=move_to.position.y+default_pose.position.y;
	com.deltaPose.position.z=move_to.position.z+default_pose.position.z;
	com.deltaPose.orientation.x=move_to.orientation.x+default_pose.orientation.x;
	com.deltaPose.orientation.y=move_to.orientation.y+default_pose.orientation.y;
	com.deltaPose.orientation.z=move_to.orientation.z+default_pose.orientation.z;
	com.deltaPose.orientation.w=move_to.orientation.w+default_pose.orientation.w;
	if(mode==1)
		Desired_pose_node.publish(com);
	if(mode==2)
		Control_node.publish(manual);
	if(mode==3)
	{
		manual.torque.z=ctrl_yaw;
		Control_node.publish(manual);
	}
}

double modulo(double f) // not a modulo but dealing with the discontinuity around pi and -pi
{
  if(f>M_PI*(1+0.05)) // +0.05 to create an hysteresis
  {
   f=f-2*M_PI;
  }
  else
  {
    if(f<-M_PI*(1+0.05))
    {
      f=f+2*M_PI;
    }
  }
  return f;
}

inline float IIR(float old, float in, float gain)
{
  return old + (1.0-gain)*(in-old);
}

void compasscallback(const sensors::compass::ConstPtr& compass)
{
	if(yaw_start)
	{
		yaw_ref=compass->rz[0];
		yaw_start=false;
	}
	t_yaw=ros::Time::now().toSec()-t_yaw;
	double err = IIR( err_yaw_old, modulo((yaw_ref-compass->rz[0])/573.5), 0.2);  // compass.rz is in degree*10 573=180*10/3.14
	double vel = IIR( vel_yaw_old, (err-err_yaw_old)/t_yaw,0.4);
	t_yaw=ros::Time::now().toSec();
	
	ctrl_yaw=1.0*err + 4*vel;
	err_yaw_old=err;
	vel_yaw_old=vel;
}



void joycallback(const sensor_msgs::Joy::ConstPtr& Joy)
{
//move_to.orientation.w=0; //this is the safety number a bit useless


move_to.position.x += bool_input(Joy->buttons[PS3_BUTTON_CROSS_RIGHT],Joy->buttons[PS3_BUTTON_CROSS_LEFT])/100.00;
move_to.position.y += bool_input(Joy->buttons[PS3_BUTTON_CROSS_UP],Joy->buttons[PS3_BUTTON_CROSS_DOWN])/100.00;
move_to.position.z += bool_input(Joy->buttons[PS3_BUTTON_REAR_RIGHT_2],Joy->buttons[PS3_BUTTON_REAR_LEFT_2])/100.00;
manual.force.x += bool_input(Joy->buttons[PS3_BUTTON_CROSS_UP],Joy->buttons[PS3_BUTTON_CROSS_DOWN])/100.00;
manual.force.y += bool_input(Joy->buttons[PS3_BUTTON_CROSS_LEFT],Joy->buttons[PS3_BUTTON_CROSS_RIGHT])/100.00;
manual.force.z += bool_input(Joy->buttons[PS3_BUTTON_REAR_RIGHT_2],Joy->buttons[PS3_BUTTON_REAR_LEFT_2])/100.00;

if (Joy->buttons[PS3_BUTTON_ACTION_CROSS]){move_to.orientation.z +=1/100.00; manual.torque.z+=1/100.0000000; yaw_ref= (yaw_ref +5) % 1800;}

if (Joy->buttons[PS3_BUTTON_ACTION_CIRCLE]){move_to.orientation.z +=-1/100.000; manual.torque.z-=1/100.000000;  yaw_ref= (yaw_ref -5) % 1800;}

if (Joy->buttons[PS3_BUTTON_ACTION_TRIANGLE])
{
	if(ros::Time::now().toSec()-temp_magn>0.25)
	{
		if(magnet_on.data){magnet_on.data=false;} 
		else {magnet_on.data=true;} 
		Magnet.publish(magnet_on);
		temp_magn=ros::Time::now().toSec();
	} 
}
if (Joy->buttons[PS3_BUTTON_START])
{
move_to.position.x=0;
move_to.position.y=0;
move_to.position.z=0;
move_to.orientation.x=0;
move_to.orientation.y=0;
move_to.orientation.w=0;
move_to.orientation.z=0;
manual.force.x=0;
manual.force.y=0;
manual.force.z=0;
manual.torque.x=0;
manual.torque.y=0;
manual.torque.z=0;
}

if (Joy->buttons[PS3_BUTTON_SELECT])
{
move_to.position.x=0;
move_to.position.y=0;
move_to.orientation.x=0;
move_to.orientation.y=0;
move_to.orientation.w=0;
move_to.orientation.z=0;
manual.force.x=0;
manual.force.y=0;
manual.torque.x=0;
manual.torque.y=0;
manual.torque.z=0;
}

sendit();
ROS_INFO("x:%f, y:%f, z:%f \n", move_to.position.x,move_to.position.y,move_to.position.z);
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "ps3");
  ros::NodeHandle n;
  ros::NodeHandle nh1("~");

  move_to.position.x=0;
  move_to.position.y=0;
  move_to.position.z=0;
  move_to.orientation.x=0;
  move_to.orientation.y=0;
  move_to.orientation.z=0;
  move_to.orientation.w=0;
  default_pose.position.x=3;
  default_pose.position.y=0;
  default_pose.position.z=3;
  default_pose.orientation.x=0;
  default_pose.orientation.y=0;
  default_pose.orientation.z=0;
  default_pose.orientation.w=0;
  manual.force.x = 0;
  manual.force.y = 0;
  manual.force.z = 0;
  manual.torque.x = 0;
  manual.torque.y = 0;
  manual.torque.z = 0;
  
  magnet_on.data=false;


  ros::Subscriber  joy_stick = n.subscribe<sensor_msgs::Joy>("/joy",10,&joycallback);
  ros::Subscriber  compass = n.subscribe<sensors::compass>("compass",10,&compasscallback);

  Control_node = n.advertise<geometry_msgs::Wrench>("command_control",1);
  Desired_pose_node = n.advertise<controls::Commands>("commands",1);
  Magnet = n.advertise<std_msgs::Bool>("magnet_on",1);

  //ros::Publisher  to_control = n.advertise<geometry_msgs::Pose>("ps3_control",10);
  ros::Rate loop_rate(20);

    if (nh1.getParam("mode", mode))
      ROS_INFO("Got param: %i", mode );



    ///set up to send to new control node
    com.header.stamp=ros::Time::now();
    com.deltaPose=move_to;
    com.onOff=true;
    com.commandOnOff=true;
    com.ctrlNb=3;
    com.maxThrust=50;
    com.GainCP=0.3;
    com.noInt=true;

while (ros::ok())
{
  ros::spinOnce();
  loop_rate.sleep();
}

return 0;
}
