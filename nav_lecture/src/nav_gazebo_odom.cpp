#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"
#include "gazebo_msgs/ModelStates.h"
#include "tf/transform_broadcaster.h"

#include "math.h"
#include <string>
#include <random>

double sdlab_uniform(){
double ret = ((double) rand() + 1.0) / ((double) RAND_MAX + 2.0);
return ret;
}
double sdlab_normal(double mu, double sigma){
double  z = sqrt(-2.0 * log(sdlab_uniform())) * sin(2.0 * M_PI * sdlab_uniform());
return mu + sigma * z;
}

std::string model_name="gazebo_model";
std::string odom_frame="odom";
std::string base_frame="base_link";
float publish_rate=20.0;
float noise=0.0;
bool tf_enable=false;
ros::Publisher odom_pub;

void tf_publish(geometry_msgs::Pose pose0){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::poseMsgToTF(pose0,transform);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), odom_frame, base_frame));
}

nav_msgs::Odometry last_odom;
bool enable_odom=false;
void models_callback(const gazebo_msgs::ModelStates& model_msg){    
  int model_size=model_msg.name.size();
  for(int i=0;i<model_size;i++){
    if(model_msg.name[i]==model_name){
      last_odom.header.frame_id=odom_frame;
      last_odom.header.stamp=ros::Time::now();
      last_odom.child_frame_id=base_frame;
      last_odom.pose.pose=model_msg.pose[i];

      last_odom.pose.pose.position.x+=sdlab_normal(0.0, noise);
      last_odom.pose.pose.position.y+=sdlab_normal(0.0, noise);
      last_odom.pose.covariance = {
      0.5, 0, 0, 0, 0, 0,  // covariance on gps_x
      0, 0.5, 0, 0, 0, 0,  // covariance on gps_y
      0, 0, 0.5, 0, 0, 0,  // covariance on gps_z
      0, 0, 0, 0.1, 0, 0,  // large covariance on rot x
      0, 0, 0, 0, 0.1, 0,  // large covariance on rot y
      0, 0, 0, 0, 0, 0.1}; // large covariance on rot z

/*
      last_odom.twist.twist=model_msg.twist[i];
      last_odom.twist.covariance = {
      1000, 0, 0, 0, 0, 0,  // covariance on gps_x
      0, 1000, 0, 0, 0, 0,  // covariance on gps_y
      0, 0, 1000, 0, 0, 0,  // covariance on gps_z
      0, 0, 0, 1000, 0, 0,  // large covariance on rot x
      0, 0, 0, 0, 1000, 0,  // large covariance on rot y
      0, 0, 0, 0, 0, 1000}; // large covariance on rot z
*/
      enable_odom=true;
    }
  }
}

void odom_callback(const ros::TimerEvent&){
  odom_pub.publish(last_odom);
  if(tf_enable){
    tf_publish(last_odom.pose.pose);
  }
}

int main(int argc, char **argv){
  ros::init(argc, argv, "nav_model_base_publisher");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  //rosparam
  pnh.getParam("model_name",   model_name);
  pnh.getParam("odom_frame",   odom_frame);
  pnh.getParam("base_frame",   base_frame);
  pnh.getParam("publish_rate", publish_rate);
  pnh.getParam("noise",        noise);
  pnh.getParam("tf_enable",    tf_enable);

  //publisher
  odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 1);

  //subscriibe
  ros::Subscriber model_sub   = nh.subscribe("/gazebo/model_states", 10, models_callback);

  //timer
  ros::Timer odom_timer = nh.createTimer(ros::Duration(1.0/publish_rate), odom_callback);
  ros::spin();
  return 0;
}
