#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <mcptam/PoseGeneratorConfig.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <TooN/TooN.h>

tf::Transform transform;

double deg2rad(double deg)
{
  return deg * M_PI / 180;
}

void callback(mcptam::PoseGeneratorConfig &config, uint32_t level) 
{        
  transform.setOrigin(tf::Point(config.x, config.y, config.z));
  tf::Quaternion q;
  if(config.rot_type == 0) // euler
  {
    q.setEulerZYX(deg2rad(config.yaw), deg2rad(config.pitch), deg2rad(config.roll));
  }
  else  // quaternion
  {
    q = tf::Quaternion(config.qx, config.qy, config.qz, config.qw);
    q.normalize();
  }
  
  transform.setRotation(q);
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "PoseGenerator");
  ros::NodeHandle nh_priv("~");
  
  std::string world_name, object_name;
  nh_priv.param<std::string>("world_name", world_name, "pose_gen_world");
  nh_priv.param<std::string>("object_name", object_name, "pose_gen_object");
  
  tf::TransformBroadcaster br;

  dynamic_reconfigure::Server<mcptam::PoseGeneratorConfig> server;
  server.setCallback(boost::bind(&callback, _1, _2));

  std::cerr<<"Run dynamic_reconfigure/reconfigure_gui to change parameters"<<std::endl;
  std::cerr<<"Broadcast frames are "<<world_name<<" and "<<object_name<<", view them in RViz"<<std::endl;
  
  ros::Rate rate(20);
  while(ros::ok())
  {
    ros::spinOnce();
    
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), world_name, object_name));
    rate.sleep();
  }
  
/*  
  TooN::SO3<> so3Rot = util::QuatToSO3(transform.getRotation());
  std::cout<<"Rotation matrix: "<<std::endl<<so3Rot<<std::endl;
  std::cout<<"Axis angle: "<<so3Rot.ln()<<std::endl;
*/
  
  return 0;
}
