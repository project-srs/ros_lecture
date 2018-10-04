#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"
#include "gazebo_msgs/ModelStates.h"
#include "tf/transform_broadcaster.h"

#include "math.h"
#include <string>
#include <iostream>
#include <sstream>
#include <random>

double sdlab_uniform()
{
  double ret = ((double) rand() + 1.0) / ((double) RAND_MAX + 2.0);
  return ret;
}
double sdlab_normal(double mu, double sigma)
{
  double  z = sqrt(-2.0 * log(sdlab_uniform())) *
    sin(2.0 * M_PI * sdlab_uniform());
 
  return mu + sigma * z;
}

std::string model_name="";
std::string world_frame="";
std::string odom_frame="";
std::string true_frame="";
float publish_rate=20.0;
float noise=0.0;
bool tf_enable=false;
ros::Publisher odom_pub;

void tf_publish(geometry_msgs::Pose pose0, std::string base_frame){
  static tf::TransformBroadcaster br;
  if(base_frame!=""){
    tf::Transform transform;
    tf::poseMsgToTF(pose0,transform);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), world_frame, base_frame));
  }
}

nav_msgs::Odometry last_odom;
geometry_msgs::Pose last_pose;
bool enable_odom=false;
void models_callback(const gazebo_msgs::ModelStates& model_msg){    
  int model_size=model_msg.name.size();
  for(int i=0;i<model_size;i++){
    if(model_msg.name[i]==model_name){
      last_odom.header.frame_id=world_frame;
	    last_odom.header.stamp=ros::Time::now();
	    last_odom.child_frame_id=odom_frame;
	    last_odom.pose.pose=model_msg.pose[i];
	    last_odom.pose.covariance = {
		    0.5, 0, 0, 0, 0, 0,  // covariance on gps_x
        0, 0.5, 0, 0, 0, 0,  // covariance on gps_y
        0, 0, 0.5, 0, 0, 0,  // covariance on gps_z
        0, 0, 0, 0.1, 0, 0,  // large covariance on rot x
        0, 0, 0, 0, 0.1, 0,  // large covariance on rot y
        0, 0, 0, 0, 0, 0.1}; // large covariance on rot z

	    last_odom.pose.pose.position.x+=sdlab_normal(0.0, noise);
	    last_odom.pose.pose.position.y+=sdlab_normal(0.0, noise);
	    
	    last_odom.twist.twist=model_msg.twist[i];
	    last_odom.twist.covariance = {
		    1000, 0, 0, 0, 0, 0,  // covariance on gps_x
        0, 1000, 0, 0, 0, 0,  // covariance on gps_y
        0, 0, 1000, 0, 0, 0,  // covariance on gps_z
        0, 0, 0, 1000, 0, 0,  // large covariance on rot x
        0, 0, 0, 0, 1000, 0,  // large covariance on rot y
        0, 0, 0, 0, 0, 1000}; // large covariance on rot z
	    
      last_pose=model_msg.pose[i];
      enable_odom=true;
	  }
  }
}

void true_callback(const ros::TimerEvent&)
{
	if(tf_enable)tf_publish(last_pose,true_frame);
}

void odom_callback(const ros::TimerEvent&)
{
	if(tf_enable){
    odom_pub.publish(last_odom);
	  tf_publish(last_odom.pose.pose,odom_frame);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "nav_model_base_publisher");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");

  //rosparam
  pn.getParam("model_name",   model_name);
  pn.getParam("world_frame",  world_frame);
  pn.getParam("odom_frame",   odom_frame);
  pn.getParam("true_frame",   true_frame);
  pn.getParam("publish_rate", publish_rate);
  pn.getParam("noise",        noise);
  pn.getParam("tf_enable",    tf_enable);

  //publisher
  odom_pub = n.advertise<nav_msgs::Odometry>("odom", 10);

  //subscriibe
  ros::Subscriber joy_sub   = n.subscribe("/gazebo/model_states", 10, models_callback);

  //timer
  ros::Timer timer1 = n.createTimer(ros::Duration(0.1), true_callback);
  ros::Timer timer2 = n.createTimer(ros::Duration(1.0/publish_rate), odom_callback);
	ros::spin();
/*
  ros::Rate loop_rate(publish_rate); 
  while (ros::ok()){
	if(enable_odom){
	  odom_pub.publish(last_odom);
	  if(tf_enable){
      tf_publish(last_odom.pose.pose,odom_frame);
      tf_publish(last_pose,true_frame);
    }
	}
	ros::spinOnce();
	loop_rate.sleep();
  }
*/ 
  return 0;
}
