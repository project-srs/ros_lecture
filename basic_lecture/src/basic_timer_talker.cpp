#include <ros/ros.h>
#include <std_msgs/String.h>

ros::Publisher chatter_pub;
void timer_callback(const ros::TimerEvent& e)
{
  std_msgs::String msg;
  msg.data = "hello world!";
  ROS_INFO("publish: %s", msg.data.c_str());
  chatter_pub.publish(msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "basic_timer_talker");
  ros::NodeHandle nh;
  chatter_pub = nh.advertise<std_msgs::String>("chatter", 10);

  ros::Timer timer = nh.createTimer(ros::Duration(0.1), timer_callback);
  ros::spin();
  return 0;
}
