/**
 * @file /include/gui_control/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef gui_control_QNODE_HPP_
#define gui_control_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Wrench.h"
#include "std_msgs/Int32.h"
#include <QThread>
#include <QStringListModel>


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace gui_control {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();
	void setmode(int x);
	void setstate(int x, int y, int z, int xx, int yy, int zz, int ww);

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);

Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();

private:
	int init_argc;
	char** init_argv;
	ros::Publisher chatter_publisher;
	ros::Publisher state_publisher;
	ros::Publisher mode_publisher;
	ros::Publisher wrench_publisher;
    QStringListModel logging_model;
    std_msgs::Int32 mode;
    geometry_msgs::Pose state;
    geometry_msgs::Wrench wrench;
     
    
};

}  // namespace gui_control

#endif /* gui_control_QNODE_HPP_ */
