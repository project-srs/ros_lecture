#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <string>
#include <math.h>

class Converter
{
public:
  Converter(void): nh_(), pnh_("~")
  {
    joy_sub_ = nh_.subscribe("/tracker", 10, &Converter::odomCallback, this);
  }
  void odomCallback(const nav_msgs::Odometry& msg)
  {
    std::string frame_id;

    std::string parent_frame_id = "";
    pnh_.getParam("parent_frame_id", parent_frame_id);
    if (parent_frame_id != ""){
      frame_id = parent_frame_id;
    } else {
      frame_id = msg.header.frame_id;
    }

    tf::Transform transform;
    tf::poseMsgToTF(msg.pose.pose, transform);
    br_.sendTransform(tf::StampedTransform(transform, msg.header.stamp, frame_id, msg.child_frame_id));
  }
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
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
