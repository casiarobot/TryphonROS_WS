#ifndef ROSEIGEN
#define ROSEIGEN

namespace roseigen{

geometry_msgs::PoseStamped pose2rospos(Eigen::VectorXd pose)
{
    geometry_msgs::Pose pose;
    pose.position.x=pose(0);
    pose.position.y=pose(1);
    pose.position.z=pose(2);

    Pose.pose.orientation.x=pose(3);
    Pose.pose.orientation.y=pose(4);
    Pose.pose.orientation.z=pose(5);

    return Pose;
}

geometry_msgs::Twist vel2twistros(Eigen::VectorXd vel)
{
      geometry_msgs::Twist twist;
      twist.linear.x=vel(0);
      twist.linear.y=vel(1);
      twist.linear.z=vel(2);

      twist.angular.x=vel(3);
      twist.angular.y=vel(4);
      twist.angular.z=vel(5);

      return twist;
}

Eigen::VectorXd twistros2vel(Eigen::VectorXd vel)
{
      geometry_msgs::Twist twist;
      twist.linear.x=vel(0);
      twist.linear.y=vel(1);
      twist.linear.z=vel(2);

      twist.angular.x=vel(3);
      twist.angular.y=vel(4);
      twist.angular.z=vel(5);

      return twist;
}


}

#endif // ROSEIGEN

