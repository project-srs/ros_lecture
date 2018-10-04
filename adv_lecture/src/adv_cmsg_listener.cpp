#include "ros/ros.h"
#include "basic_lecture/cmsg.h"

void chatterCallback(const basic_lecture::cmsg& msg){
  ROS_INFO("I heard: [%s, %u]", msg.word.c_str(),msg.number);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "basic_listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("chatter", 10, chatterCallback);
  ros::Rate loop_rate(10);

  ros::spin();
  return 0;
}

