#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <turtlesim/Spawn.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class ListenTest
{
public:
  ListenTest() : nh_(), tfBuffer_(), tfListener_(tfBuffer_)
  {
    timer_ = nh_.createTimer(ros::Duration(0.5), [&](const ros::TimerEvent& e) {
      geometry_msgs::TransformStamped transformStamped;
      try
      {
        transformStamped = tfBuffer_.lookupTransform("base_link", "dynamic_tf", ros::Time(0));
      }
      catch (tf2::TransformException& ex)
      {
        ROS_WARN("%s", ex.what());
        return;
      }
      auto& trans = transformStamped.transform.translation;
      ROS_INFO("world->dynamic_tf: %f %f %f", trans.x, trans.y, trans.z);

      geometry_msgs::Pose object_d, object_w;
      object_d.position.z = 1.0;
      object_d.orientation.w = 1.0;
      tf2::doTransform(object_d, object_w, transformStamped);
      ROS_INFO("object_w x:%f, y:%f, z:%f", object_w.position.x, object_w.position.y, object_w.position.z);
    });
  }

private:
  ros::NodeHandle nh_;
  ros::Timer timer_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "my_tf2_listener");
  ListenTest listen_test;
  ros::spin();
  return 0;
};