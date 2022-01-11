#include <ros/ros.h>
#include <cstdio>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

class BroadCasterTest
{
public:
  BroadCasterTest() : nh_()
  {
    broadcast_static_tf();
    timer_ = nh_.createTimer(ros::Duration(0.1), [&](const ros::TimerEvent& e) {
      broadcast_dynamic_tf();
      counter_++;
    });
  }

private:
  void broadcast_static_tf(void)
  {
    geometry_msgs::TransformStamped static_transformStamped;
    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = "base_link";
    static_transformStamped.child_frame_id = "static_tf";
    static_transformStamped.transform.translation.x = 0.0;
    static_transformStamped.transform.translation.y = 0.0;
    static_transformStamped.transform.translation.z = 0.3;
    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, 0.0);
    static_transformStamped.transform.rotation.x = quat.x();
    static_transformStamped.transform.rotation.y = quat.y();
    static_transformStamped.transform.rotation.z = quat.z();
    static_transformStamped.transform.rotation.w = quat.w();
    static_br_.sendTransform(static_transformStamped);
  }
  void broadcast_dynamic_tf(void)
  {
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "base_link";
    transformStamped.child_frame_id = "dynamic_tf";
    transformStamped.transform.translation.x = 0.3 * cos(counter_ * 0.1);
    transformStamped.transform.translation.y = 0.3 * sin(counter_ * 0.1);
    transformStamped.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, 0.0);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    dynamic_br_.sendTransform(transformStamped);
  }
  ros::NodeHandle nh_;
  ros::Timer timer_;
  tf2_ros::TransformBroadcaster dynamic_br_;
  tf2_ros::StaticTransformBroadcaster static_br_;
  int counter_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "my_static_tf2_broadcaster");
  BroadCasterTest broadcast_test;
  ros::spin();
  return 0;
};