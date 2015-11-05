



#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
// Euler file
#include <Euler_utility.h>
#include "vects2geoMsgs.cpp"
#include <math.h>
#include <ctime>

 Eigen::Vector3d Gazpos,Gaz1pos_243a,Gaz2pos_243a, Gaz1pos_244a, Gaz2pos_244a,Gaz1pos_243b,Gaz2pos_243b, Gaz1pos_244b, Gaz2pos_244b, Gaz1vel_243, Gaz2vel_243, Gaz1vel_244 ,Gaz2vel_244;
 Eigen::Vector3d Gazangle,Gaz1angle_243,Gaz2angle_243, Gaz1angle_244, Gaz2angle_244, Gaz1avel_243, Gaz2avel_243, Gaz1avel_244, Gaz2avel_244;
Eigen::Matrix3d RMatrix, Rmatrix1_243,Rmatrix2_243,Rmatrix1_244,Rmatrix2_244, RvelM;
geometry_msgs::PoseStamped  Gazp1a, Gazp2a,Gazp1b, Gazp2b;
geometry_msgs::TwistStamped Gazv1, Gazv2;
double t;



void subgaz1_243a(const geometry_msgs::PoseStamped Pose)
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
 
  Gaz1pos_243a(0)=Gazposition(0);
  Gaz1pos_243a(1)=Gazposition(1);
  Gaz1pos_243a(2)=Gazposition(2);

}

void subgaz2_243a(const geometry_msgs::PoseStamped Pose)
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
 
  Gaz2pos_243a(0)=Gazposition(0);
  Gaz2pos_243a(1)=Gazposition(1);
  Gaz2pos_243a(2)=Gazposition(2);

}

void subgaz1_244a(const geometry_msgs::PoseStamped Pose)
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

  Gaz1angle_244(0)=atan2(2*(quat.w()*quat.x()+quat.y()*quat.z()),1-2*(quat.x()*quat.x()+quat.y()*quat.y()));//+.02*sin(750*t);
  Gaz1angle_244(1)=asin(2*(quat.w()*quat.y()-quat.z()*quat.x()));//+.03*sin(1000*t);
  Gaz1angle_244(2)=atan2(2*(quat.w()*quat.z()+quat.x()*quat.y()),1-2*(quat.z()*quat.z()+quat.y()*quat.y()));//+.025*sin(550*t);
 
  Gaz1pos_244a(0)=Gazposition(0);//+.05*sin(560*t);
  Gaz1pos_244a(1)=Gazposition(1);//+.04*sin(940*t);
  Gaz1pos_244a(2)=Gazposition(2);//+.045*sin(810*t);

}

void subgaz2_244a(const geometry_msgs::PoseStamped Pose)
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
 
  Gaz2pos_244a(0)=Gazposition(0);
  Gaz2pos_244a(1)=Gazposition(1);
  Gaz2pos_244a(2)=Gazposition(2);

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

  Gaz1avel_243(0)=-Twist.twist.angular.x;  //(-) like applying quatang[0001] rotation to get 0 orientation when facing
  Gaz1avel_243(1)=-Twist.twist.angular.y;
  Gaz1avel_243(2)=Twist.twist.angular.z;
 
  Gaz1vel_243(0)=Gazvel(0);//+.008*sin(460*t); 
  Gaz1vel_243(1)=Gazvel(1);//+.0042*sin(760*t);
  Gaz1vel_243(2)=Gazvel(2);//+.0093*sin(910*t);
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

  Gaz1avel_244(0)=-Twist.twist.angular.x;  //(-) like applying quatang[0001] rotation to get 0 orientation when facing
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
void subgaz1_243b(const geometry_msgs::PoseStamped Pose)
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

//Eigen::Quaterniond quattemp=quat.inverse();
//Rmatrix1_243=quattemp.toRotationMatrix();
//Gaz1position=Rmatrix1*Gaz1position;

 
  Gaz1pos_243b(0)=Gazposition(0);
  Gaz1pos_243b(1)=Gazposition(1);
  Gaz1pos_243b(2)=Gazposition(2);

}

void subgaz2_243b(const geometry_msgs::PoseStamped Pose)
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

//Eigen::Quaterniond quattemp=quat.inverse();
//Rmatrix2_243=quattemp.toRotationMatrix();
//Gaz1position=Rmatrix1*Gaz1position;

 
  Gaz2pos_243b(0)=Gazposition(0);
  Gaz2pos_243b(1)=Gazposition(1);
  Gaz2pos_243b(2)=Gazposition(2);

}

void subgaz1_244b(const geometry_msgs::PoseStamped Pose)
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

//Eigen::Quaterniond quattemp=quat.inverse();
//Rmatrix1_244=quattemp.toRotationMatrix();
//Gaz1position=Rmatrix1*Gaz1position;

Eigen::Quaterniond quatang(0,0,0,1); //to make orientation all 0 when docking face alligned
quat=quat*quatang.inverse();  
 
  Gaz1pos_244b(0)=Gazposition(0);//+.05*sin(560*t);
  Gaz1pos_244b(1)=Gazposition(1);//+.04*sin(940*t);
  Gaz1pos_244b(2)=Gazposition(2);//+.045*sin(810*t);

}

void subgaz2_244b(const geometry_msgs::PoseStamped Pose)
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

//Eigen::Quaterniond quattemp=quat.inverse();
//Rmatrix2_244=quattemp.toRotationMatrix();

Eigen::Quaterniond quatang(0,0,0,1); //to make orientation all 0 when docking face alligned
quat=quat*quatang.inverse();  
RvelM=quatang.toRotationMatrix();

 
  Gaz2pos_244b(0)=Gazposition(0);
  Gaz2pos_244b(1)=Gazposition(1);
  Gaz2pos_244b(2)=Gazposition(2);

}

////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////
///////////////
///////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////
//////////////////////
////////////////
////////////////
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
  t=ros::Time::now().toSec();
//ros::Subscriber sub_243 = nh.subscribe("/192_168_10_243/poseStamped_gazebo",1,subgaz_243);
//ros::Subscriber sub_244 = nh.subscribe("/192_168_10_244/poseStamped_gazebo",1,subgaz_244);
ros::Subscriber sub1_243a = nh.subscribe("/192_168_10_243/poseStamped_gazebo_1a",1,subgaz1_243a);
ros::Subscriber sub2_243a = nh.subscribe("/192_168_10_243/poseStamped_gazebo_2a",1,subgaz2_243a);
ros::Subscriber sub1_244a = nh.subscribe("/192_168_10_244/poseStamped_gazebo_1a",1,subgaz1_244a);
ros::Subscriber sub2_244a = nh.subscribe("/192_168_10_244/poseStamped_gazebo_2a",1,subgaz2_244a);
ros::Subscriber subv1_243a = nh.subscribe("/192_168_10_243/velocity_1",1,subgazv1_243);
ros::Subscriber subv2_243a = nh.subscribe("/192_168_10_243/velocity_2",1,subgazv2_243);
ros::Subscriber subv1_244a = nh.subscribe("/192_168_10_244/velocity_1",1,subgazv1_244);
ros::Subscriber subv2_244a = nh.subscribe("/192_168_10_244/velocity_2",1,subgazv2_244);

ros::Subscriber sub1_243b = nh.subscribe("/192_168_10_243/poseStamped_gazebo_1b",1,subgaz1_243b);
ros::Subscriber sub2_243b = nh.subscribe("/192_168_10_243/poseStamped_gazebo_2b",1,subgaz2_243b);
ros::Subscriber sub1_244b = nh.subscribe("/192_168_10_244/poseStamped_gazebo_1b",1,subgaz1_244b);
ros::Subscriber sub2_244b = nh.subscribe("/192_168_10_244/poseStamped_gazebo_2b",1,subgaz2_244b);

ros::Publisher  Gazp1_puba = nh.advertise<geometry_msgs::PoseStamped>("/192_168_10_243/ar_posea",1);
ros::Publisher  Gazp2_puba = nh.advertise<geometry_msgs::PoseStamped>("/192_168_10_244/ar_posea",1);
ros::Publisher  Gazp1_pubb = nh.advertise<geometry_msgs::PoseStamped>("/192_168_10_243/ar_poseb",1);
ros::Publisher  Gazp2_pubb = nh.advertise<geometry_msgs::PoseStamped>("/192_168_10_244/ar_poseb",1);
ros::Publisher  Gazv1_pub = nh.advertise<geometry_msgs::TwistStamped>("/192_168_10_243/ar_vel",1);
ros::Publisher  Gazv2_pub = nh.advertise<geometry_msgs::TwistStamped>("/192_168_10_244/ar_vel",1);




ros::Rate loop_rate(10);

pose_zero(Gazp1a);
pose_zero(Gazp2a);
pose_zero(Gazp1b);
pose_zero(Gazp2b);

twist_zero(Gazv1);
twist_zero(Gazv2);



while (ros::ok())
 {

ros::spinOnce();





Gazp1a.header.stamp=ros::Time::now();
Gazp2a.header.stamp=ros::Time::now();
Gazp1b.header.stamp=ros::Time::now();
Gazp2b.header.stamp=ros::Time::now();
Gazp1a.pose=vects2pose(Rmatrix1_243*(Gaz2pos_244a-Gaz1pos_243a),(Gaz1angle_244-Gaz2angle_243));
Gazp2a.pose=vects2pose(Rmatrix2_243*(Gaz1pos_244a-Gaz2pos_243a),(Gaz2angle_244-Gaz1angle_243));
Gazp1b.pose=vects2pose(Rmatrix1_243*(Gaz2pos_244b-Gaz1pos_243a),(Gaz1angle_244-Gaz2angle_243));
Gazp2b.pose=vects2pose(Rmatrix2_243*(Gaz1pos_244b-Gaz2pos_243a),(Gaz2angle_244-Gaz1angle_243));



Gazv1.header.stamp=ros::Time::now();
Gazv2.header.stamp=ros::Time::now();
Gazv1.twist=vects2twist(Rmatrix1_243*(Gaz2vel_244-Gaz1vel_243),(Gaz1avel_244-Gaz2avel_243));
Gazv2.twist=vects2twist(Rmatrix2_243*(Gaz1vel_244-Gaz2vel_243),(Gaz2avel_244-Gaz1avel_243));



  	Gazp1_puba.publish(Gazp1a);
  	Gazp2_puba.publish(Gazp2a);
  	Gazv1_pub.publish(Gazv1);
  	Gazv2_pub.publish(Gazv2);
    Gazp1_pubb.publish(Gazp1b);
    Gazp2_pubb.publish(Gazp2b);

loop_rate.sleep();
}

	return 0;
}

