#include "ros/ros.h"
#include "std_msgs/String.h"

int HZ=10;
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("SUB:%s:%f", msg->data.c_str(),ros::Time::now().toSec());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "basic_timing_listener");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
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

