#include <dynamic_reconfigure/server.h>
#include <std_msgs/Bool.h>
#include <controls/magnetConfig.h>
#include "controls.h"

ros::Publisher magnet_chaser;
ros::Publisher magnet_target;
std_msgs::Bool magnet_on;
void callback(controls::magnetConfig &config, uint32_t level) 
{
	ROS_INFO("Reconfigure Request: magnet");


bool magnet_clicker=true;
magnet_clicker=config.magnet;

if(magnet_clicker) // i think this will work if it is a one time signal that needs to be sent
{
	magnet_on.data=true;
magnet_chaser.publish(magnet_on);
magnet_target.publish(magnet_on);	

}
else 
{
	magnet_on.data=true;

magnet_chaser.publish(magnet_on);
magnet_target.publish(magnet_on);

}

}


int main(int argc, char **argv)
{

ros::init(argc, argv, "magnet");
ros::NodeHandle nh;

magnet_chaser=nh.advertise<std_msgs::Bool>("/192_168_10_241/magnet_on",1);
magnet_target=nh.advertise<std_msgs::Bool>("/192_168_10_244/magnet_on",1);

dynamic_reconfigure::Server<controls::magnetConfig> server;
dynamic_reconfigure::Server<controls::magnetConfig>::CallbackType f;

 f = boost::bind(&callback, _1, _2);
 server.setCallback(f);

ros::Rate loop_rate(10);

	while (ros::ok())
	 {

	ros::spinOnce();
	
	

 loop_rate.sleep();

 }
return 0;
}