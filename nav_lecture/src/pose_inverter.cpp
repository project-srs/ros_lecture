#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>

#include "geometry_util.h"

class PoseInverter{
public:
  PoseInverter() : nh_(), pnh_("~") {
    odom_sub_ = nh_.subscribe("odom", 10, &PoseInverter::odomCallback, this);
    pose_sub_ = nh_.subscribe("pose", 10, &PoseInverter::poseCallback, this);
  }

  void odomCallback(const nav_msgs::Odometry& msg) {
    last_odom_ = msg;
  }

  void poseCallback(const geometry_msgs::PoseStamped& msg) {
    geometry_msgs::PoseStamped output;
    output.header.stamp = msg.header.stamp;
    std::string child_frame_id = msg.header.frame_id;
    output.header.frame_id = last_odom_.header.frame_id;
    output.pose = relay(last_odom_.pose.pose, invert(msg.pose));
    broadcast_dynamic_tf(output, child_frame_id);
  }

private:

  void broadcast_dynamic_tf(geometry_msgs::PoseStamped pose, std::string child_frame_id)
  {
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header = pose.header;
    transformStamped.child_frame_id = child_frame_id;
    transformStamped.transform.translation.x = pose.pose.position.x;
    transformStamped.transform.translation.y = pose.pose.position.y;
    transformStamped.transform.translation.z = pose.pose.position.z;
    transformStamped.transform.rotation = pose.pose.orientation;
    dynamic_br_.sendTransform(transformStamped);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber odom_sub_;
  nav_msgs::Odometry last_odom_;
  ros::Subscriber pose_sub_;
  tf2_ros::TransformBroadcaster dynamic_br_;
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "pose_inverter");
  PoseInverter pose_inverter;
  ros::spin();
  return 0;
}
