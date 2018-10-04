#include "ros/ros.h"
#include "std_msgs/String.h"

int HZ=10;
int main(int argc, char **argv){
  ros::init(argc, argv, "basic_timing_talker");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  pn.getParam("HZ",  HZ);
	ros::Rate loop_rate(HZ);

  int count = 0;
  while (ros::ok()){
    char word[10];
    sprintf(word,"%03i",count);
    std_msgs::String msg;
    msg.data = word;
    ROS_INFO("PUB:%03i:%f", count, ros::Time::now().toSec());
    chatter_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  return 0;
}

