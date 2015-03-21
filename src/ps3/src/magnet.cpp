#include  <ros/ros.h>


int main(int argc, char **argv)
{
ros::init(argc, argv, "magnet");
ros::NodeHandle node;

ros::Publisher magnet=node.advertise<std_msgs::Bool>("/magnet_on",1);

std_msgs::Bool m;

m.data="true";

magnet.publish(m);


return 0;
}
