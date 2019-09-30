#include <ros/ros.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "do_dishes_client");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  int param_data=0;
  pnh.getParam("int_param", param_data);
  ROS_INFO("[%s] param:%i", ros::this_node::getName().c_str(), param_data);

  ros::spin();
  return 0;
}