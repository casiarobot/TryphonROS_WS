/*
This node was "coded" by py the 13/11/2015
It is made to be use with two leddars and the compas
mode 0 : Manual
mode 1 : Control X and Z
mode 2 : whatever you want
*/

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Wrench.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>


#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Float64.h"
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

#include "ps3values.h"

#define PI 3.14159265



ros::Publisher  pose_des;

geometry_msgs::Wrench  move_to;
geometry_msgs::Wrench wrenchMsg;
geometry_msgs::Pose go_to; //used with MCPTAM to go to a location
geometry_msgs::Pose old_go_to;

int mode=0; // 1 for mcptam pose
int print=0;

//bool start=true;
float kz=0.0015;

float incx=0,incy=0,incz=0, incyaw=0;  //for open loop control
float posx=0,posy=0,posz=0; //for pose
std_msgs::Bool button_magnet;
double temp_mode=0.0;


geometry_msgs::Wrench zeroWrench()
{
    geometry_msgs::Wrench w;
    w.force.x=0;
    w.force.y=0;
    w.force.y=0;
    w.torque.x=0;
    w.torque.y=0;
    w.torque.z=0;
    return w;
}

geometry_msgs::Pose zeroPose()
{
    geometry_msgs::Pose p;
    p.position.x=0;
    p.position.y=0;
    p.position.z=0;
    p.orientation.x=0;
    p.orientation.y=0;
    p.orientation.z=0;
    p.orientation.w=0;
    return p;
}

geometry_msgs::Wrench wrench;

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

    float button_x=0, button_y=0, button_z=0, button_yaw=0;

    if(Joy->buttons[PS3_BUTTON_ACTION_SQUARE])
    {
        if(ros::Time::now().toSec()-temp_mode>0.25)
        {
            if(mode == 1){ mode =0;}
            else { mode =1;}
            temp_mode=ros::Time::now().toSec();
        }
    }

    button_x =bool_input(Joy->buttons[PS3_BUTTON_CROSS_UP],Joy->buttons[PS3_BUTTON_CROSS_DOWN]);
    button_y = bool_input(Joy->buttons[PS3_BUTTON_CROSS_LEFT],Joy->buttons[PS3_BUTTON_CROSS_RIGHT]);
    button_z =bool_input(Joy->buttons[PS3_BUTTON_REAR_RIGHT_2],Joy->buttons[PS3_BUTTON_REAR_LEFT_2]);
    button_yaw =bool_input(Joy->buttons[PS3_BUTTON_ACTION_CROSS],Joy->buttons[PS3_BUTTON_ACTION_CIRCLE]);


    if (mode==0)
    {
        if((incx<maxFX && button_x>0)||(incx>minFX && button_x<0)){incx=incx+button_x;}
        if((incy<maxFY && button_y>0)||(incy>minFY && button_y<0)){incy=incy+button_y;}
        if((incz<maxFZ && button_z>0)||(incz>minFZ && button_z<0)){incz=incz+button_z;}
        if((incyaw<maxTYaw && button_yaw>0)||(incyaw>minTYaw && button_yaw<0)){incyaw=incyaw+button_yaw;}
    }
    else
    {
        //increment x with max and min for pose
        if((posx<maxX && button_x>0)||(posx>minX && button_x<0)){posx=posx+button_x;}
        //increment y with max and min for pose
        if((posy<maxY && button_y>0)||(posy>minY && button_y<0)){posy=posy+button_y;}
        //increment z with max and min for pose
        if((posz<maxZ && button_z>0)||(posz>minZ && button_z<0)){posz=posz+button_z;}


    }

    //auto shut off x and y
    if (Joy->buttons[PS3_BUTTON_SELECT])
    {
        incx=0;
        incy=0;
        incyaw=0;
    }
    // auto shut off all values
    if (Joy->buttons[PS3_BUTTON_START])
    {
        incx=0;
        incy=0;
        incz=0;
        incyaw=0;
        posx = 0;
        posz = 0;
    }


    wrench.force.x=incx/100.0;
    wrench.force.y=incy/100.0;
    wrench.force.z=incz/25.0;
    wrench.torque.z=incyaw/100.0;

    go_to.position.x=posx/100.0;
    go_to.position.y=posy/100.0;
    go_to.position.z=posz/100.0;

    if(mode==0)
    {
        go_to = zeroPose();
    }
}

void myocallback(const std_msgs::Float64 i)
{
    if(i.data > 0.5){posy -= 1;}
    if(i.data < 0.5){posy += 1;}
    go_to.position.y=posy/100.0;
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "ps3_z");
    ros::NodeHandle n;

    mode = 1;

    wrench = zeroWrench();
    go_to = zeroPose();


    ros::Subscriber joy_stick = n.subscribe<sensor_msgs::Joy>("joy",1,&joycallback);
    ros::Subscriber myo = n.subscribe<std_msgs::Float64>("myo_info/float",1,myocallback);


    ros::Publisher pose_des = n.advertise<geometry_msgs::Pose>("desired_deltapose",1);
    ros::Publisher to_control = n.advertise<geometry_msgs::Wrench>("ps3_control",1);
    ros::Publisher mode_control = n.advertise<std_msgs::Int64>("control_mode",1);
    ros::Rate loop_rate(20);

    while (ros::ok())
    {
        std_msgs::Int64 modeMsg;
        modeMsg.data = mode;
        mode_control.publish(modeMsg);
        ROS_INFO("mode : %i", mode);
        if(mode==0)
        {
            to_control.publish(wrench);
        }
        else
        {
            pose_des.publish(go_to);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
