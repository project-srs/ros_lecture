#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <string>
#include <math.h>

class Converter
{
public:
  Converter(void)
  {
    joy_sub_ = nh_.subscribe("/tracker", 10, &Converter::odomCallback, this);
  }
  void odomCallback(const nav_msgs::Odometry& msg)
  {
    tf::Transform transform;
    tf::poseMsgToTF(msg.pose.pose, transform);
    br_.sendTransform(tf::StampedTransform(transform, msg.header.stamp, /*msg.header.frame_id*/ "odom", msg.child_frame_id));
  }
  ros::NodeHandle nh_;
  ros::Subscriber joy_sub_;
  tf::TransformBroadcaster br_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sim1_odom_tf_converter");
  Converter converter;
  ros::spin();
  return 0;
}