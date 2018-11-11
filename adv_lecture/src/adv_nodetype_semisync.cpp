#include "ros/ros.h"
#include "std_msgs/Float32.h"

float input2_last=0;
void input2_callback(const std_msgs::Float32& float_msg){
  input2_last=float_msg.data;
}

ros::Publisher output_pub;
void input1_callback(const std_msgs::Float32& float_msg){
  std_msgs::Float32 float_data;
  float_data.data = float_msg.data+input2_last;
  output_pub.publish(float_data);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "adv_nodetype_semisync");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  output_pub = n.advertise<std_msgs::Float32>("output", 10);
	ros::Subscriber input1_sub= n.subscribe("/input1", 10, input1_callback); 
	ros::Subscriber input2_sub= n.subscribe("/input2", 10, input2_callback); 

	ros::spin();
  return 0;
}
