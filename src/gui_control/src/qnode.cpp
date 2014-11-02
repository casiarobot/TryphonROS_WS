/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/gui_control/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace gui_control {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{
	mode.data = 0;
	state.position.x = 0;
	state.position.y = 0;
	state.position.z = 0;
	state.orientation.x = 0;
	state.orientation.y = 0;
	state.orientation.z = 0;
	state.orientation.w = 0;
	wrench.force.x=0;
	wrench.force.y=0;
	wrench.force.z=0;
	wrench.torque.x=0;
	wrench.torque.y=0;
	wrench.torque.z=0;
		}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"gui_control");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
	chatter_publisher = n.advertise<std_msgs::String>("chatter", 1);
	state_publisher = n.advertise<geometry_msgs::Pose>("desired_state", 1);
	mode_publisher = n.advertise<std_msgs::Int32>("mode", 1);
	start();
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"gui_control");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
	chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
	//start();
	return true;
}

void QNode::run() {
	ros::Rate loop_rate(20);
	int count = 0;
	int modeval=mode.data;
	while ( ros::ok() ) {

		std_msgs::String msg;
		std::stringstream ss;
		ss << "hello world " << count;
		//msg.data = ss.str();
		//chatter_publisher.publish(msg);
		mode_publisher.publish(mode);
		switch (modeval)
		{
			case 0:
			break;
			
			case 1: wrench_publisher.publish(wrench);
			break;
			
			case 2: state_publisher.publish(state);
			break;
			
			case 3: state_publisher.publish(state);
			break;
			
			default:
			break;
		}
		state_publisher.publish(state);
		//log(Info,std::string("I sent: ")+msg.data);
		ros::spinOnce();
		loop_rate.sleep();
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::setmode(int x)
{
		mode.data=x;
}
	
void QNode::setstate(int x, int y, int z, int xx, int yy, int zz, int ww)
{
	state.position.x = x;
	state.position.y = y;
	state.position.z = z;
	state.orientation.x = xx;
	state.orientation.y = yy;
	state.orientation.z = zz;
	state.orientation.w = ww;
}

}  // namespace gui_control
