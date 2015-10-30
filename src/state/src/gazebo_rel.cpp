



#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
// Euler file
#include <Euler_utility.h>
#include "vects2geoMsgs.cpp"
#include <math.h>

 Eigen::Vector3d Gazpos,Gaz1pos_243,Gaz2pos_243, Gaz1pos_244, Gaz2pos_244, Gaz1vel_243, Gaz2vel_243, Gaz1vel_244 ,Gaz2vel_244;
 Eigen::Vector3d Gazangle,Gaz1angle_243,Gaz2angle_243, Gaz1angle_244, Gaz2angle_244, Gaz1avel_243, Gaz2avel_243, Gaz1avel_244, Gaz2avel_244;
Eigen::Matrix3d RMatrix, Rmatrix1_243,Rmatrix2_243,Rmatrix1_244,Rmatrix2_244, RvelM;
geometry_msgs::PoseStamped  Gazp1, Gazp2;
geometry_msgs::TwistStamped Gazv1, Gazv2;




void subgaz1_243(const geometry_msgs::PoseStamped Pose)
{

  Eigen::Vector3d Gazposition;

  Gazposition(0)=Pose.pose.position.x; // defined in global frame
  Gazposition(1)=Pose.pose.position.y;
  Gazposition(2)=Pose.pose.position.z;
  Eigen::Quaterniond quat(Pose.pose.orientation.w,Pose.pose.orientation.x,Pose.pose.orientation.y,Pose.pose.orientation.z);
  
//ROS_INFO("Pose.z=%f",Pose.pose.position.z);

//Eigen::Quaterniond quatpos(0,0,.7071,-.7071); //to make x y z of camera/artag output aloligned with tryphon body frame
//Eigen::Quaterniond quatpos(.7071,-.7071,0,0);
//Eigen::Quaterniond quatang(0,1,0,0); //to make orientation all 0 when docking face alligned
//quat=quat*quatang.inverse();  

Eigen::Quaterniond quattemp=quat.inverse();
Rmatrix1_243=quattemp.toRotationMatrix();
//Gaz1position=Rmatrix1*Gaz1position;

  Gaz1angle_243(0)=atan2(2*(quat.w()*quat.x()+quat.y()*quat.z()),1-2*(quat.x()*quat.x()+quat.y()*quat.y()));
  Gaz1angle_243(1)=asin(2*(quat.w()*quat.y()-quat.z()*quat.x()));
  Gaz1angle_243(2)=atan2(2*(quat.w()*quat.z()+quat.x()*quat.y()),1-2*(quat.z()*quat.z()+quat.y()*quat.y()));
 
  Gaz1pos_243(0)=Gazposition(0);
  Gaz1pos_243(1)=Gazposition(1);
  Gaz1pos_243(2)=Gazposition(2);

}

void subgaz2_243(const geometry_msgs::PoseStamped Pose)
{

  Eigen::Vector3d Gazposition;

  Gazposition(0)=Pose.pose.position.x; // defined in global frame
  Gazposition(1)=Pose.pose.position.y;
  Gazposition(2)=Pose.pose.position.z;
  Eigen::Quaterniond quat(Pose.pose.orientation.w,Pose.pose.orientation.x,Pose.pose.orientation.y,Pose.pose.orientation.z);
  
//ROS_INFO("Pose.z=%f",Pose.pose.position.z);

//Eigen::Quaterniond quatpos(0,0,.7071,-.7071); //to make x y z of camera/artag output aloligned with tryphon body frame
//Eigen::Quaterniond quatpos(.7071,-.7071,0,0);
//Eigen::Quaterniond quatang(0,1,0,0); //to make orientation all 0 when docking face alligned
//quat=quat*quatang.inverse();  

Eigen::Quaterniond quattemp=quat.inverse();
Rmatrix2_243=quattemp.toRotationMatrix();
//Gaz1position=Rmatrix1*Gaz1position;

  Gaz2angle_243(0)=atan2(2*(quat.w()*quat.x()+quat.y()*quat.z()),1-2*(quat.x()*quat.x()+quat.y()*quat.y()));
  Gaz2angle_243(1)=asin(2*(quat.w()*quat.y()-quat.z()*quat.x()));
  Gaz2angle_243(2)=atan2(2*(quat.w()*quat.z()+quat.x()*quat.y()),1-2*(quat.z()*quat.z()+quat.y()*quat.y()));
 
  Gaz2pos_243(0)=Gazposition(0);
  Gaz2pos_243(1)=Gazposition(1);
  Gaz2pos_243(2)=Gazposition(2);

}

void subgaz1_244(const geometry_msgs::PoseStamped Pose)
{

  Eigen::Vector3d Gazposition;

  Gazposition(0)=Pose.pose.position.x; // defined in global frame
  Gazposition(1)=Pose.pose.position.y;
  Gazposition(2)=Pose.pose.position.z;
  Eigen::Quaterniond quat(Pose.pose.orientation.w,Pose.pose.orientation.x,Pose.pose.orientation.y,Pose.pose.orientation.z);
  
//ROS_INFO("Pose.z=%f",Pose.pose.position.z);

//Eigen::Quaterniond quatpos(0,0,.7071,-.7071); //to make x y z of camera/artag output aloligned with tryphon body frame
//Eigen::Quaterniond quatpos(.7071,-.7071,0,0);
//Eigen::Quaterniond quatang(0,1,0,0); //to make orientation all 0 when docking face alligned
//quat=quat*quatang.inverse();  

Eigen::Quaterniond quattemp=quat.inverse();
Rmatrix1_244=quattemp.toRotationMatrix();
//Gaz1position=Rmatrix1*Gaz1position;

Eigen::Quaterniond quatang(0,0,0,1); //to make orientation all 0 when docking face alligned
quat=quat*quatang.inverse();  

  Gaz1angle_244(0)=atan2(2*(quat.w()*quat.x()+quat.y()*quat.z()),1-2*(quat.x()*quat.x()+quat.y()*quat.y()));
  Gaz1angle_244(1)=asin(2*(quat.w()*quat.y()-quat.z()*quat.x()));
  Gaz1angle_244(2)=atan2(2*(quat.w()*quat.z()+quat.x()*quat.y()),1-2*(quat.z()*quat.z()+quat.y()*quat.y()));
 
  Gaz1pos_244(0)=Gazposition(0);
  Gaz1pos_244(1)=Gazposition(1);
  Gaz1pos_244(2)=Gazposition(2);

}

void subgaz2_244(const geometry_msgs::PoseStamped Pose)
{

  Eigen::Vector3d Gazposition;

  Gazposition(0)=Pose.pose.position.x; // defined in global frame
  Gazposition(1)=Pose.pose.position.y;
  Gazposition(2)=Pose.pose.position.z;
  Eigen::Quaterniond quat(Pose.pose.orientation.w,Pose.pose.orientation.x,Pose.pose.orientation.y,Pose.pose.orientation.z);
  
//ROS_INFO("Pose.z=%f",Pose.pose.position.z);

//Eigen::Quaterniond quatpos(0,0,.7071,-.7071); //to make x y z of camera/artag output aloligned with tryphon body frame
//Eigen::Quaterniond quatpos(.7071,-.7071,0,0);

//Eigen::Quaterniond quatang(0,1,0,0); //to make orientation all 0 when docking face alligned
//quat=quat*quatang.inverse();  

Eigen::Quaterniond quattemp=quat.inverse();
Rmatrix2_244=quattemp.toRotationMatrix();

Eigen::Quaterniond quatang(0,0,0,1); //to make orientation all 0 when docking face alligned
quat=quat*quatang.inverse();  
RvelM=quatang.toRotationMatrix();


  Gaz2angle_244(0)=atan2(2*(quat.w()*quat.x()+quat.y()*quat.z()),1-2*(quat.x()*quat.x()+quat.y()*quat.y()));
  Gaz2angle_244(1)=asin(2*(quat.w()*quat.y()-quat.z()*quat.x()));
  Gaz2angle_244(2)=atan2(2*(quat.w()*quat.z()+quat.x()*quat.y()),1-2*(quat.z()*quat.z()+quat.y()*quat.y()));
 
  Gaz2pos_244(0)=Gazposition(0);
  Gaz2pos_244(1)=Gazposition(1);
  Gaz2pos_244(2)=Gazposition(2);

}

////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////
///////////////
///////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////


void subgazv1_243(const geometry_msgs::TwistStamped Twist)
{

  Eigen::Vector3d Gazvel;

  Gazvel(0)=Twist.twist.linear.x; // defined in global frame
  Gazvel(1)=Twist.twist.linear.y;
  Gazvel(2)=Twist.twist.linear.z;
  
//ROS_INFO("Pose.z=%f",Pose.pose.position.z);

//Eigen::Quaterniond quatpos(0,0,.7071,-.7071); //to make x y z of camera/artag output aloligned with tryphon body frame
//Eigen::Quaterniond quatpos(.7071,-.7071,0,0);
//Eigen::Quaterniond quatang(0,1,0,0); //to make orientation all 0 when docking face alligned
//quat=quat*quatang.inverse();  
//Gaz1position=Rmatrix1*Gaz1position;

  Gaz1avel_243(0)=-Twist.twist.angular.x;
  Gaz1avel_243(1)=-Twist.twist.angular.y;
  Gaz1avel_243(2)=Twist.twist.angular.z;
 
  Gaz1vel_243(0)=Gazvel(0); 
  Gaz1vel_243(1)=Gazvel(1);
  Gaz1vel_243(2)=Gazvel(2);
}

void subgazv2_243(const geometry_msgs::TwistStamped Twist)
{

  Eigen::Vector3d Gazvel;

  Gazvel(0)=Twist.twist.linear.x; // defined in global frame
  Gazvel(1)=Twist.twist.linear.y;
  Gazvel(2)=Twist.twist.linear.z;
  
//ROS_INFO("Pose.z=%f",Pose.pose.position.z);

//Eigen::Quaterniond quatpos(0,0,.7071,-.7071); //to make x y z of camera/artag output aloligned with tryphon body frame
//Eigen::Quaterniond quatpos(.7071,-.7071,0,0);
//Eigen::Quaterniond quatang(0,1,0,0); //to make orientation all 0 when docking face alligned
//quat=quat*quatang.inverse();  
//Gaz1position=Rmatrix1*Gaz1position;

  Gaz2avel_243(0)=-Twist.twist.angular.x;
  Gaz2avel_243(1)=-Twist.twist.angular.y;
  Gaz2avel_243(2)=Twist.twist.angular.z;


  Gaz2vel_243(0)=Gazvel(0); 
  Gaz2vel_243(1)=Gazvel(1);
  Gaz2vel_243(2)=Gazvel(2);
}
void subgazv1_244(const geometry_msgs::TwistStamped Twist)
{

  Eigen::Vector3d Gazvel;

  Gazvel(0)=Twist.twist.linear.x; // defined in global frame
  Gazvel(1)=Twist.twist.linear.y;
  Gazvel(2)=Twist.twist.linear.z;
  
//ROS_INFO("Pose.z=%f",Pose.pose.position.z);

//Eigen::Quaterniond quatpos(0,0,.7071,-.7071); //to make x y z of camera/artag output aloligned with tryphon body frame
//Eigen::Quaterniond quatpos(.7071,-.7071,0,0);
//Eigen::Quaterniond quatang(0,1,0,0); //to make orientation all 0 when docking face alligned
//quat=quat*quatang.inverse();  
//Gaz1position=Rmatrix1*Gaz1position;

  Gaz1avel_244(0)=-Twist.twist.angular.x;
  Gaz1avel_244(1)=-Twist.twist.angular.y;
  Gaz1avel_244(2)=Twist.twist.angular.z;
 
//Gaz1avel_244=RvelM*Gaz1avel_244;

  Gaz1vel_244(0)=Gazvel(0); 
  Gaz1vel_244(1)=Gazvel(1);
  Gaz1vel_244(2)=Gazvel(2);
}

void subgazv2_244(const geometry_msgs::TwistStamped Twist)
{

  Eigen::Vector3d Gazvel;

  Gazvel(0)=Twist.twist.linear.x; // defined in global frame
  Gazvel(1)=Twist.twist.linear.y;
  Gazvel(2)=Twist.twist.linear.z;
  
//ROS_INFO("Pose.z=%f",Pose.pose.position.z);

//Eigen::Quaterniond quatpos(0,0,.7071,-.7071); //to make x y z of camera/artag output aloligned with tryphon body frame
//Eigen::Quaterniond quatpos(.7071,-.7071,0,0);
//Eigen::Quaterniond quatang(0,1,0,0); //to make orientation all 0 when docking face alligned
//quat=quat*quatang.inverse();  
//Gaz1position=Rmatrix1*Gaz1position;

  Gaz2avel_244(0)=-Twist.twist.angular.x;
  Gaz2avel_244(1)=-Twist.twist.angular.y;
  Gaz2avel_244(2)=Twist.twist.angular.z;
 
//Gaz2avel_244=RvelM*Gaz2avel_244;

  Gaz2vel_244(0)=Gazvel(0); 
  Gaz2vel_244(1)=Gazvel(1);
  Gaz2vel_244(2)=Gazvel(2);
}
////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////
///////////////
///////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////

void pose_zero(geometry_msgs::PoseStamped &p) //set any pose msg to zero
{
	p.pose.position.x=0;
	p.pose.position.y=0;
	p.pose.position.z=0;
	p.pose.orientation.w=0; 
	p.pose.orientation.x=0;
	p.pose.orientation.y=0;
	p.pose.orientation.z=0;
}

void twist_zero(geometry_msgs::TwistStamped &tw) //set any pose msg to zero
{
	tw.twist.linear.x=0;
	tw.twist.linear.y=0;
	tw.twist.linear.z=0;
	tw.twist.angular.x=0;
	tw.twist.angular.y=0;
	tw.twist.angular.z=0;
}



int main(int argc, char **argv)
{


  ros::init(argc, argv, "gazebo_rel");
  ros::NodeHandle nh;

//ros::Subscriber sub_243 = nh.subscribe("/192_168_10_243/poseStamped_gazebo",1,subgaz_243);
//ros::Subscriber sub_244 = nh.subscribe("/192_168_10_244/poseStamped_gazebo",1,subgaz_244);
ros::Subscriber sub1_243 = nh.subscribe("/192_168_10_243/poseStamped_gazebo_1",1,subgaz1_243);
ros::Subscriber sub2_243 = nh.subscribe("/192_168_10_243/poseStamped_gazebo_2",1,subgaz2_243);
ros::Subscriber sub1_244 = nh.subscribe("/192_168_10_244/poseStamped_gazebo_1",1,subgaz1_244);
ros::Subscriber sub2_244 = nh.subscribe("/192_168_10_244/poseStamped_gazebo_2",1,subgaz2_244);
ros::Subscriber subv1_243 = nh.subscribe("/192_168_10_243/velocity_1",1,subgazv1_243);
ros::Subscriber subv2_243 = nh.subscribe("/192_168_10_243/velocity_2",1,subgazv2_243);
ros::Subscriber subv1_244 = nh.subscribe("/192_168_10_244/velocity_1",1,subgazv1_244);
ros::Subscriber subv2_244 = nh.subscribe("/192_168_10_244/velocity_2",1,subgazv2_244);


ros::Publisher  Gazp1_pub = nh.advertise<geometry_msgs::PoseStamped>("/192_168_10_243/ar_pose",1);
ros::Publisher  Gazp2_pub = nh.advertise<geometry_msgs::PoseStamped>("/192_168_10_244/ar_pose",1);
ros::Publisher  Gazv1_pub = nh.advertise<geometry_msgs::TwistStamped>("/192_168_10_243/ar_vel",1);
ros::Publisher  Gazv2_pub = nh.advertise<geometry_msgs::TwistStamped>("/192_168_10_244/ar_vel",1);




ros::Rate loop_rate(10);

pose_zero(Gazp1);
pose_zero(Gazp2);
twist_zero(Gazv1);
twist_zero(Gazv2);



while (ros::ok())
 {

ros::spinOnce();





Gazp1.header.stamp=ros::Time::now();
Gazp2.header.stamp=ros::Time::now();
Gazp1.pose=vects2pose(Rmatrix1_243*(Gaz2pos_244-Gaz1pos_243),(Gaz1angle_244-Gaz2angle_243));
Gazp2.pose=vects2pose(Rmatrix2_243*(Gaz1pos_244-Gaz2pos_243),(Gaz2angle_244-Gaz1angle_243));



Gazv1.header.stamp=ros::Time::now();
Gazv2.header.stamp=ros::Time::now();
Gazv1.twist=vects2twist(Rmatrix1_243*(Gaz2vel_244-Gaz1vel_243),(Gaz1avel_244-Gaz2avel_243));
Gazv2.twist=vects2twist(Rmatrix2_243*(Gaz1vel_244-Gaz2vel_243),(Gaz2avel_244-Gaz1avel_243));



  	Gazp1_pub.publish(Gazp1);
  	Gazp2_pub.publish(Gazp2);
  	Gazv1_pub.publish(Gazv1);
  	Gazv2_pub.publish(Gazv2);

}

	return 0;
}

