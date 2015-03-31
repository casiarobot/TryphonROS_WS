#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Wrench.h"

#include <Eigen/Geometry>
#include <Eigen/Dense>

void wrench_zero(geometry_msgs::Wrench &w)
{
  w.force.x=0;
  w.force.y=0;
  w.force.z=0;
  w.torque.x=0;
  w.torque.y=0;
  w.torque.z=0;
}

void vect3_zero(Eigen::Vector3d &vect)
{
  vect(0)=0;
  vect(1)=0;
  vect(2)=0;
}

void vect4_zero(Eigen::Vector4d &vect)
{
  vect(0)=1;
  vect(1)=0;
  vect(2)=0;
  vect(3)=0;
}

geometry_msgs::Pose vects2pose(Eigen::Vector3d position,Eigen::Vector3d angle)
{
  geometry_msgs::Pose pose;

  pose.position.x=position(0);
  pose.position.y=position(1);
  pose.position.z=position(2);
  pose.orientation.x=angle(0);
  pose.orientation.y=angle(1);
  pose.orientation.z=angle(2);

  return pose;
}

geometry_msgs::Twist vects2twist(Eigen::Vector3d v,Eigen::Vector3d av)
{
  geometry_msgs::Twist twist;

  twist.linear.x=v(0);
  twist.linear.y=v(1);
  twist.linear.z=v(2);
  twist.angular.x=av(0);
  twist.angular.y=av(1);
  twist.angular.z=av(2);

  return twist;
}

geometry_msgs::Wrench vects2wrench(Eigen::Vector3d f,Eigen::Vector3d t)
{
  geometry_msgs::Wrench wrench;

  wrench.force.x=f(0);
  wrench.force.y=f(1);
  wrench.force.z=f(2);
  wrench.torque.x=t(0);
  wrench.torque.y=t(1);
  wrench.torque.z=t(2);

  return wrench;
}

Eigen::Vector3d pose2vect_pos(geometry_msgs::Pose p)
{
  Eigen::Vector3d position;

  position(0)=p.position.x;
  position(1)=p.position.y;
  position(2)=p.position.z;

  return position;
}

Eigen::Vector3d pose2vect_angle(geometry_msgs::Pose p)
{
  Eigen::Vector3d orientation;

  orientation(0)=p.orientation.x;
  orientation(1)=p.orientation.y;
  orientation(2)=p.orientation.z;

  return orientation;
}


Eigen::Vector3d twist2vect_linear(geometry_msgs::Twist t)
{
  Eigen::Vector3d linear;

  linear(0)=t.linear.x;
  linear(1)=t.linear.y;
  linear(2)=t.linear.z;

  return linear;
}

Eigen::Vector3d twist2vect_angular(geometry_msgs::Twist t)
{
  Eigen::Vector3d angular;

  angular(0)=t.angular.x;
  angular(1)=t.angular.y;
  angular(2)=t.angular.z;

  return angular;
}

Eigen::Vector3d wrench2vect_force(geometry_msgs::Wrench w)
{
  Eigen::Vector3d f;

  f(0)=w.force.x;
  f(1)=w.force.y;
  f(2)=w.force.z;

  return f;
}

Eigen::Vector3d wrench2vect_torque(geometry_msgs::Wrench w)
{
  Eigen::Vector3d t;

  t(0)=w.torque.x;
  t(1)=w.torque.y;
  t(2)=w.torque.z;

  return t;
}


