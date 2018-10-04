#include "ros/ros.h"
#include "std_msgs/String.h"

#include "ros/ros.h"
#include "std_msgs/String.h"

ros::Publisher chatter_pub;
int number=0;
int HZ=10;
 
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("N%02i:%s:%f", number, msg->data.c_str(),ros::Time::now().toSec());
  chatter_pub.publish(msg);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "basic_timing_bridge");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  chatter_pub = n.advertise<std_msgs::String>("chatter_out", 1000);
  ros::Subscriber chatter_sub = n.subscribe("chatter_in", 1000, chatterCallback);
  pn.getParam("number",  number);
	pn.getParam("HZ",  HZ);

  if(HZ>0){
  	ros::Rate loop_rate(HZ);
    while (ros::ok()){
      ros::spinOnce();
      loop_rate.sleep();
    }
  }
  else ros::spin();
  return 0;
}

