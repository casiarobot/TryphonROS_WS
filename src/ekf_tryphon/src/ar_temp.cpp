#include "ros/ros.h"
#include "std_msgs/String.h"
#include <tf/transform_broadcaster.h>

// Standard C/C++ libraries
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <ctime>
#include <iostream>
#include "kalmanbasefilter.h"
#include <cmath>
#include <limits>

// ROS messages libraries
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"

//libraries for the sonars and the compass
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <leddartech/leddar_data.h>

// Eigen libraries
#include <Eigen/Geometry>
#include <Eigen/Dense>

// Euler file
#include <Euler_utility.h>

// Kalman file
#include <Kalman_utility.h>

#include <sstream>

// Dynamic reconfigure


Eigen::Quaterniond quatposAR(.7071,-.7071,0,0);
Eigen::Quaterniond quattempAR=quatposAR; //no camera angle

Eigen::Matrix3d RmatrixAR=quattempAR.toRotationMatrix();
Eigen::Matrix3d qm1,qm2;
Eigen::Quaterniond quat1,quat2;
Eigen::Vector3d ea1,ea2;
//will need to be fixed
geometry_msgs::PoseStamped alvarpose1B,alvarpose2B,alvarpose1S,alvarpose2S,alvarfinal1,alvarfinal2,compasscheating;
bool id_2=false,id_3=false,id_0=false,id_1=false,id_4=false,id_5=false;

void subArTag1B(const ar_track_alvar_msgs::AlvarMarkers Aposes1)
{
	bool both;
    ar_track_alvar_msgs::AlvarMarker alvartemp1,alvartemp2;
    if(Aposes1.markers.size() > 0)
    {
        
        for (int c=0;c<Aposes1.markers.size();c++)
        {
            
            if(Aposes1.markers[c].id==2 && !id_2){alvartemp1=Aposes1.markers[c]; id_2=true; } //can change id's accordingly
            else if (Aposes1.markers[c].id==3 && !id_3){alvartemp2=Aposes1.markers[c]; id_3=true;}
        }

        if(id_2 && id_3){both=true;}
    }
    else {ROS_INFO("no big tag seen 1");return;}

if(id_3)
{
    alvarpose1B.pose=alvartemp2.pose.pose;
    
    //make changes for right position
}
else if(id_2 && !both)
{
   alvarpose1B.pose=alvartemp1.pose.pose;

}
}

void subArTag2B(const ar_track_alvar_msgs::AlvarMarkers Aposes2)
{
	bool both;
    ar_track_alvar_msgs::AlvarMarker alvartemp1,alvartemp2;
    if(Aposes2.markers.size() > 0)
    {
        
        for (int c=0;c<Aposes2.markers.size();c++)
        {
            
            if(Aposes2.markers[c].id==0 && !id_0){alvartemp1=Aposes2.markers[c]; id_0=true; } //can change id's accordingly
            else if (Aposes2.markers[c].id==1 && !id_1){alvartemp2=Aposes2.markers[c]; id_1=true;}
        }

        if(id_0 && id_1){both=true;}
    }
    else {ROS_INFO("no big tag seen 2 ");return;}

if(id_0)
{
alvarpose2B.pose=alvartemp1.pose.pose;
}
else if(id_1 && !both)
{
    alvarpose2B.pose=alvartemp2.pose.pose;
    //make changes for right position
}
}

void subArTag1S(const ar_track_alvar_msgs::AlvarMarkers Aposes1)
{
	
    ar_track_alvar_msgs::AlvarMarker alvartemp1;
    if(Aposes1.markers.size() > 0)
    {
        
        for (int c=0;c<Aposes1.markers.size();c++)
        {
            
            if(Aposes1.markers[c].id==4 && !id_4){alvartemp1=Aposes1.markers[c]; id_4=true; } //can change id's accordingly         
        }
    }
    else {ROS_INFO("no small tag seen 1 ");return;}

if(id_4)
{
    alvarpose1S.pose=alvartemp1.pose.pose;
    //make changes for right position
}
}

void subArTag2S(const ar_track_alvar_msgs::AlvarMarkers Aposes2)
{
	
    ar_track_alvar_msgs::AlvarMarker alvartemp1;
    if(Aposes2.markers.size() > 0)
    {
        
        for (int c=0;c<Aposes2.markers.size();c++)
        {
            
            if(Aposes2.markers[c].id==5 && !id_5){alvartemp1=Aposes2.markers[c]; id_5=true; } //can change id's accordingly         
        }
    }
    else {ROS_INFO("no small tag seen 2 ");return;}

if(id_5)
{
    alvarpose2S.pose=alvartemp1.pose.pose;
    //make changes for right position
}


}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ar_temp");
    ros::NodeHandle n;
    Eigen::Vector3d ALVARposition1,ALVARposition2;
    //  Get Params



    ros::Subscriber subAr1B = n.subscribe("/192_168_10_241/artags1/artag_big/ar_pose_marker",1, subArTag1B);
    ros::Subscriber subAr2B = n.subscribe("/192_168_10_242/artags2/artag_big/ar_pose_marker",1, subArTag2B);
    ros::Subscriber subAr1S = n.subscribe("/192_168_10_241/artags1/artag_small/ar_pose_marker",1, subArTag1S);
    ros::Subscriber subAr2S = n.subscribe("/192_168_10_242/artags2/artag_small/ar_pose_marker",1, subArTag2S);

      //Publishers //
    ros::Publisher pubAR1 = n.advertise<geometry_msgs::PoseStamped>("/192_168_10_241/artemp_pose1",1);
    ros::Publisher pubAR2 = n.advertise<geometry_msgs::PoseStamped>("/192_168_10_242/artemp_pose2",1);
    ros::Publisher pubcompcheat = n.advertise<geometry_msgs::PoseStamped>("/192_168_10_242/compasscheat",1);
ros::Rate loop_rate(10);

    while (ros::ok())
    {
    
      
    
        if(id_4)
        {
        	alvarfinal1=alvarpose1S;
        	ALVARposition1(0)=alvarfinal1.pose.position.x;
        	ALVARposition1(1)=alvarfinal1.pose.position.y;
        	ALVARposition1(2)=alvarfinal1.pose.position.z;
        	ALVARposition1=RmatrixAR*ALVARposition1;
            quat1.w()=alvarfinal1.pose.orientation.w;
            quat1.x()=alvarfinal1.pose.orientation.x;
            quat1.y()=alvarfinal1.pose.orientation.y;
            quat1.z()=alvarfinal1.pose.orientation.z;
            qm1=RmatrixAR.transpose()*quat1.toRotationMatrix();
             ea1=qm1.eulerAngles(3,2,1);

        }

        else  if(id_3 && !id_4)
        {
        	alvarfinal1=alvarpose1B;
        	ALVARposition1(0)=alvarfinal1.pose.position.x;
        	ALVARposition1(1)=alvarfinal1.pose.position.y;
        	ALVARposition1(2)=alvarfinal1.pose.position.z;
        	ALVARposition1=RmatrixAR*ALVARposition1;
        	ALVARposition1(0)=ALVARposition1(0)-.125;
        	ALVARposition1(1)=ALVARposition1(1);
        	ALVARposition1(2)=ALVARposition1(2)+.082;

            quat1.w()=alvarfinal1.pose.orientation.w;
            quat1.x()=alvarfinal1.pose.orientation.x;
            quat1.y()=alvarfinal1.pose.orientation.y;
            quat1.z()=alvarfinal1.pose.orientation.z;
            qm1=RmatrixAR.transpose()*quat1.toRotationMatrix();
             ea1=qm1.eulerAngles(3,2,1);
        }
        else if(id_2 && !id_4 && !id_3)
        {
        	alvarfinal1=alvarpose1B;
        	ALVARposition1(0)=alvarfinal1.pose.position.x;  //11.5 4.0 8.5
        	ALVARposition1(1)=alvarfinal1.pose.position.y;
        	ALVARposition1(2)=alvarfinal1.pose.position.z;
        	ALVARposition1=RmatrixAR*ALVARposition1;
        	ALVARposition1(0)=ALVARposition1(0)-.125;
        	ALVARposition1(1)=ALVARposition1(1);
        	ALVARposition1(2)=ALVARposition1(2)+.170;
            quat1.w()=alvarfinal1.pose.orientation.w;
            quat1.x()=alvarfinal1.pose.orientation.x;
            quat1.y()=alvarfinal1.pose.orientation.y;
            quat1.z()=alvarfinal1.pose.orientation.z;
            qm1=RmatrixAR.transpose()*quat1.toRotationMatrix();
             ea1=qm1.eulerAngles(3,2,1);
        }

       	alvarfinal1.pose.position.x=ALVARposition1(0);
		alvarfinal1.pose.position.y=ALVARposition1(1);
		alvarfinal1.pose.position.z=ALVARposition1(2);
        alvarfinal1.pose.orientation.x=ea1(2);
        alvarfinal1.pose.orientation.y=ea1(1);
        alvarfinal1.pose.orientation.z=ea1(0);

  		if(id_5)
        {
        	alvarfinal2=alvarpose2S;
        	ALVARposition2(0)=alvarfinal2.pose.position.x;
        	ALVARposition2(1)=alvarfinal2.pose.position.y;
        	ALVARposition2(2)=alvarfinal2.pose.position.z;
        	ALVARposition2=RmatrixAR*ALVARposition2;
             quat2.w()=alvarfinal2.pose.orientation.w;
            quat2.x()=alvarfinal2.pose.orientation.x;
            quat2.y()=alvarfinal2.pose.orientation.y;
            quat2.z()=alvarfinal2.pose.orientation.z;
            qm2=RmatrixAR.transpose()*quat2.toRotationMatrix();
             ea2=qm2.eulerAngles(3,2,1);
        }
        else  if(id_0 && !id_5)
        {
        	alvarfinal2=alvarpose2B;
        	ALVARposition2(0)=alvarfinal2.pose.position.x;
        	ALVARposition2(1)=alvarfinal2.pose.position.y;
        	ALVARposition2(2)=alvarfinal2.pose.position.z;
        	ALVARposition2=RmatrixAR*ALVARposition2;
        	ALVARposition2(0)=ALVARposition2(0)+.12;
        	ALVARposition2(1)=ALVARposition2(1);
        	ALVARposition2(2)=ALVARposition2(2)+.075;
            quat2.w()=alvarfinal2.pose.orientation.w;
            quat2.x()=alvarfinal2.pose.orientation.x;
            quat2.y()=alvarfinal2.pose.orientation.y;
            quat2.z()=alvarfinal2.pose.orientation.z;
            qm2=RmatrixAR.transpose()*quat2.toRotationMatrix();
             ea2=qm2.eulerAngles(3,2,1);
        }
        else if(id_1 && !id_0 && !id_5)
        { 
        	alvarfinal2=alvarpose2B;
        	ALVARposition2(0)=alvarfinal2.pose.position.x;
        	ALVARposition2(1)=alvarfinal2.pose.position.y;
        	ALVARposition2(2)=alvarfinal2.pose.position.z;
        	ALVARposition2=RmatrixAR*ALVARposition2;
        	ALVARposition2(0)=ALVARposition2(0)+.12;
        	ALVARposition2(1)=ALVARposition2(1);
        	ALVARposition2(2)=ALVARposition2(2)+.165;
            quat2.w()=alvarfinal2.pose.orientation.w;
            quat2.x()=alvarfinal2.pose.orientation.x;
            quat2.y()=alvarfinal2.pose.orientation.y;
            quat2.z()=alvarfinal2.pose.orientation.z;
            qm2=RmatrixAR.transpose()*quat2.toRotationMatrix();
             ea2=qm2.eulerAngles(3,2,1);
        }

       	alvarfinal2.pose.position.x=ALVARposition2(0);
		alvarfinal2.pose.position.y=ALVARposition2(1);
		alvarfinal2.pose.position.z=ALVARposition2(2);
        alvarfinal2.pose.orientation.x=ea2(2);
        alvarfinal2.pose.orientation.y=ea2(1);
        alvarfinal2.pose.orientation.z=ea2(0);




if( (!id_4 && !id_3 && !id_2) || (!id_0 && !id_5 && !id_1) )
{
compasscheating.pose.orientation.z=compasscheating.pose.orientation.z;//asin((alvarfinal2.pose.position.y-alvarfinal1.pose.position.y)/1.90);
}
else
{
compasscheating.pose.orientation.z=asin((alvarfinal2.pose.position.y-alvarfinal1.pose.position.y)/1.90);
}


id_0=false;
id_1=false;
id_2=false;
id_3=false; //try making old id's and make the old ones false here but new ones false up
id_4=false;
id_5=false;
    
compasscheating=alvarfinal1;
compasscheating.pose.orientation.z=asin((alvarfinal2.pose.position.y-alvarfinal1.pose.position.y)/1.90);
//compasscheating.pose.orientation.z=(alvarfinal1.pose.orientation.z+alvarfinal2.pose.orientation.z-3.14)/2.0;
pubAR1.publish(alvarfinal1);
pubAR2.publish(alvarfinal2);
pubcompcheat.publish(compasscheating);

  ros::spinOnce();
    loop_rate.sleep();
    
    }


  return 0;
}