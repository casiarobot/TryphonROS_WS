#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Wrench.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensors/compass.h>
#include <controls/Commands.h>
#include <std_msgs/Bool.h>
#include "sensors/leddar.h"
#include "sensors/leddarArray.h"
#include <ctime>
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
double temp_mode=0;
int yaw_ref=0;
double err_yaw_old=0;
double vel_yaw_old=0;
bool yaw_start=true;
bool z_start=true;
double ctrl_yaw=0;
double ctrl_z=0;
double t_yaw=0;
double dsxt[5]={0,0,0,0,0};
double dsyt[5]={0,0,0,0,0};
double dszt[5]={0,0,0,0,0};
double dsrt[5]={0,0,0,0,0};
double dsxtf[5]={0,0,0,0,0};
double dsytf[5]={0,0,0,0,0};
double dsztf[5]={0,0,0,0,0};
double dsrtf[5]={0,0,0,0,0};

double dsx=0;
double dsy=0;
double dsz[5]={0,0,0,0,0};
double dsz1old=0;
double dszwant=2000;
double errorn=0;
double erroro=0;
double errorz=0;
double errorzo=0;
double deriv=0;
double derivo=0;
double derivz=0;
double vecZ=0.0141;
double intZ=0;
float kz=0.0015;
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
    manual.force.z=ctrl_z;
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
		yaw_ref=compass->rz[0]-180.0;
		yaw_start=false;
	}
	t_yaw=ros::Time::now().toSec()-t_yaw;
	double err = IIR( err_yaw_old, modulo((yaw_ref-compass->rz[0]-180.00)/57.35), 0.2);  // compass.rz is in degree*10 573=180*10/3.14
	double vel = IIR( vel_yaw_old, (err-err_yaw_old)/.1,0.4);
	t_yaw=ros::Time::now().toSec();
	
	ctrl_yaw=-0.75*err -0.05*vel;
  ROS_INFO("Avel: %f",vel);

	if (fabs(ctrl_yaw)>0.5){ctrl_yaw=copysign(0.5,ctrl_yaw);}
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
if(mode==3)
  {
    dszwant+=bool_input(Joy->buttons[PS3_BUTTON_REAR_RIGHT_2],Joy->buttons[PS3_BUTTON_REAR_LEFT_2])/5.0;
  }



if (Joy->buttons[PS3_BUTTON_ACTION_CROSS]){move_to.orientation.z +=1/200.00; manual.torque.z+=1/200.0000000; yaw_ref= (yaw_ref +1) % 180;} //ratio changes from 100->200 for laval new motors

if (Joy->buttons[PS3_BUTTON_ACTION_CIRCLE]){move_to.orientation.z +=-1/100.000; manual.torque.z-=1/100.000000;  yaw_ref= (yaw_ref -1) % 180;}

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

if (Joy->buttons[PS3_BUTTON_ACTION_SQUARE])
{
	if(ros::Time::now().toSec()-temp_mode>0.25)
	{
		if(mode==2)
      {mode=3;
        deriv=0;
        derivo=0;
        errorn=0;
        erroro=0;
         intZ=40;
         ctrl_z=0.14;
         ctrl_yaw=0;
        yaw_start=true;
        z_start=true;
        manual.force.z=0;
        manual.torque.z=0;
      } 
		else if(mode==3){mode=2;}
		temp_mode=ros::Time::now().toSec();
		ROS_INFO("change mode");
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


ROS_INFO("x:%f, y:%f, z:%f, yaw:%f, mode:%i ", move_to.position.x,move_to.position.y,move_to.position.z,move_to.orientation.z, mode  );
ROS_INFO("x:%f, y:%f, z:%f, yaw:%f, mode:%i,  dszwant:%f", manual.force.x,manual.force.y,manual.force.z,manual.torque.z+ ctrl_yaw , mode ,dszwant );
}

void subLeddar(const sensors::leddarArray::ConstPtr& msg)
{

  double dsx1 = 0, dsy1 = 0, dsz1 = 0;
  for(int i = 0; i < 4;i++){
    dsxt[i] = dsxt[i+1];
    dsxtf[i] = dsxtf[i+1];
    dsyt[i] = dsyt[i+1];
    dsytf[i] = dsytf[i+1];
    dszt[i] = dszt[i+1];
    dsztf[i] = dsztf[i+1];
  }
  //dszto=dszt;
  for ( int i=0; i < msg->leddars.size(); ++i)
  {
    const sensors::leddar &leddar = msg->leddars[i];
    //ROS_INFO_STREAM("ID: " << sonar.id << " - D0: " << sonar.distance[0] <<
    //                 ", D1: " << sonar.distance[1]);

    // From cm to m

    
    switch (leddar.id) {
      case 0x90:
        dsx1 = leddar.distance / 100.0;
        break;
      case 0x92:
        dsy1 = leddar.distance / 100.0;
        break;
      case 0x98:
        dsz1 = leddar.distance / 100.0;
        break;
    }

  }

  if(fabs(dsz1)<0.01)
  {
    dsz1=dsz1old;
  }
  dsxt[4] = dsx1;
  dsyt[4] = dsy1;
  dszt[4] = dsz1;
  
  dsz1old=dsz1;
  // Sonars filtering
  dsxtf[4]=3.159*dsxtf[3]-3.815*dsxtf[2]+2.076*dsxtf[1]-0.4291*dsxtf[0]+0.01223*dsxt[4]-0.02416*dsxt[3]+0.03202*dsxt[2]-0.02416*dsxt[1]+0.01223*dsxt[0];
  dsytf[4]=3.159*dsytf[3]-3.815*dsytf[2]+2.076*dsytf[1]-0.4291*dsytf[0]+0.01223*dsyt[4]-0.02416*dsyt[3]+0.03202*dsyt[2]-0.02416*dsyt[1]+0.01223*dsyt[0];
  dsztf[4]=3.159*dsztf[3]-3.815*dsztf[2]+2.076*dsztf[1]-0.4291*dsztf[0]+0.01223*dszt[4]-0.02416*dszt[3]+0.03202*dszt[2]-0.02416*dszt[1]+0.01223*dszt[0];


  //ROS_INFO("distance z: %f, distance x : %f, distance y : %f",dsztf[4],dsxtf[4],dsytf[4]);

  //SONARpos = Eigen::Vector3d(dsxtf[4], dsytf[4], dsztf[4]);
  /*SONARpos(0) = dsxtf[4];
  SONARpos(1) = dsytf[4];
  SONARpos(2) = dsztf[4];*/
/*
  if(start_leddar){
    start_leddar=false;
    ROS_INFO("Leddar started!");
  }*/

  if(z_start)
  {

    dszwant=dsztf[4];
    z_start=false;
  }

        erroro=errorn;
        derivo=deriv;
        
  // State estimation //
       errorn=-(dszwant-dsztf[4]);
    //integral
     if(fabs(ctrl_z<2.4) && fabs(manual.force.x)<1.2 && fabs(manual.force.y)<1.2) // increasing only if the command is not saturating //
     {
        intZ+= errorn/10;
        if(fabs(intZ*vecZ)>0.6){intZ=copysign(0.6/vecZ,intZ);}
    }
        // Z-Axis //
        //errorn=dszwant-dsz;
        deriv=(errorn-erroro)/0.1;
   //if(fabs(deriv)>4.50){deriv=derivo;}

  ctrl_z=.04*(3.2*errorn+10.8*deriv+.15*intZ);

  //ROS_INFO("fuck: z:%f, zbf:%f, zf:%f, intZ:%f",dsz1,dszt[4],dsztf[4],intZ );
  ROS_INFO("intz:%f,error:%f,deriv:%f, ctrz_z:%f",intZ,errorn,deriv, ctrl_z);
  
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


  ros::Subscriber  joy_stick = n.subscribe<sensor_msgs::Joy>("/joy",1,&joycallback);
  ros::Subscriber  compass = n.subscribe<sensors::compass>("/192_168_10_241/compass",1,&compasscallback);
  ros::Subscriber subL = n.subscribe("/192_168_10_241/leddars", 1, subLeddar);
  Control_node = n.advertise<geometry_msgs::Wrench>("/192_168_10_243/command_control",1);
  Desired_pose_node = n.advertise<controls::Commands>("/192_168_10_243/commands",1);
  Magnet = n.advertise<std_msgs::Bool>("/192_168_10_243/magnet_on",1);

  //ros::Publisher  to_control = n.advertise<geometry_msgs::Pose>("ps3_control",10);
  ros::Rate loop_rate(10);

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
	sendit();
	ros::spinOnce();
	loop_rate.sleep();
}

return 0;
}
