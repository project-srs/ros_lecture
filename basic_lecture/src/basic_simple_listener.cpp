#include "ros/ros.h"
#include "std_msgs/String.h"

void chatterCallback(const std_msgs::String& msg){
  ROS_INFO("I heard: [%s]", msg.data.c_str());
}

int main(int argc, char **argv){
  ros::init(argc, argv, "basic_simple_listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("chatter", 10, chatterCallback);
  ros::Rate loop_rate(10);

  ros::spin();
  return 0;
}

