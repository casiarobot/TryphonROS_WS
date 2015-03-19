#include <ros/ros.h>
#include <stdlib.h>
#include "std_msgs/Int32.h"
#include <dynamic_reconfigure/server.h>
#include <mode/modeConfig.h>
std_msgs::Int32 i;


/*void callback(mode::modeConfig &config, uint32_t level) {
    ROS_INFO("Mode: %d",config.int_param);
    i.data=config.int_param;

}
*/


int main(int argc, char ** argv)
{
    if (argc==3)
    {
      ROS_INFO("TARGET IS: %s", argv[1]);
      ROS_INFO("Mode is: %s", argv[2]);
      i.data = atoi(argv[2]);

    }
    else
    {
        ROS_ERROR("Failed to get param 'target' or mode");
        return 0;
    }
    char rosname[100],ip[100];
    std::string s, temp_arg ;
    temp_arg = argv[1];
    std::replace(temp_arg.begin(), temp_arg.end(), '.', '_');
    sprintf(rosname,"mode_%s",temp_arg.c_str());


    ros::init(argc, argv, rosname);
    ros::NodeHandle node;

/*    dynamic_reconfigure::Server<mode::modeConfig> server;
    dynamic_reconfigure::Server<mode::modeConfig>::CallbackType f;

    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);
*/
    ros::Rate loop_rate(20);
    sprintf(rosname,"/%s/mode",temp_arg.c_str());
    ros::Publisher mod_pub=node.advertise<std_msgs::Int32>(rosname,1);

    while(ros::ok())
    {
        mod_pub.publish(i);
        loop_rate.sleep();
    }
    return 0;
}
