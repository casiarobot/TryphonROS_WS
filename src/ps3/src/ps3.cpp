#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <controls/Commands.h>

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
controls::Commands com;

ros::Publisher Desired_pose_node;

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
move_to.orientation.w=0; //this is the safety number


move_to.position.x = bool_input(Joy->buttons[PS3_BUTTON_CROSS_RIGHT],Joy->buttons[PS3_BUTTON_CROSS_LEFT]);
move_to.position.y = bool_input(Joy->buttons[PS3_BUTTON_CROSS_UP],Joy->buttons[PS3_BUTTON_CROSS_DOWN]);
move_to.position.z = bool_input(Joy->buttons[PS3_BUTTON_REAR_RIGHT_2],Joy->buttons[PS3_BUTTON_REAR_LEFT_2]);

if (Joy->buttons[PS3_BUTTON_ACTION_CROSS])
{
move_to.position.x=0;
move_to.position.y=0;
move_to.position.z=0;
move_to.orientation.x=0;
move_to.orientation.y=0;
move_to.orientation.z=0;
move_to.orientation.w=1;
}

if (Joy->buttons[PS3_BUTTON_ACTION_CIRCLE])
{
move_to.position.x=0;
move_to.position.y=0;
move_to.position.z=0;
move_to.orientation.x=0;
move_to.orientation.y=0;
move_to.orientation.z=0;
move_to.orientation.w=-1;
}

ROS_INFO("x:%f, y:%f, z:%f \n", move_to.position.x,move_to.position.y,move_to.position.z);
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "ps3");
  ros::NodeHandle n;

move_to.position.x=0;
move_to.position.y=0;
move_to.position.z=0;
move_to.orientation.x=0;
move_to.orientation.y=0;
move_to.orientation.z=0;
move_to.orientation.w=0;



  ros::Subscriber  joy_stick = n.subscribe<sensor_msgs::Joy>("joy",10,&joycallback);


  Desired_pose_node = n.advertise<controls::Commands>("/192_168_10_243/commands",1);

  //ros::Publisher  to_control = n.advertise<geometry_msgs::Pose>("ps3_control",10);
  ros::Rate loop_rate(20);


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
  Desired_pose_node.publish(com);
  ros::spinOnce();
  loop_rate.sleep();
}

return 0;
}
