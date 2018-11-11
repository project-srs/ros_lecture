#include "ros/ros.h"
#include "std_msgs/Float32.h"

ros::Publisher output_pub;
void input1_callback(const std_msgs::Float32& float_msg){
    std_msgs::Float32 float_data;
    float_data.data = float_msg.data;
    output_pub.publish(float_data);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "adv_nodetype_async");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  output_pub = n.advertise<std_msgs::Float32>("output", 10);
	ros::Subscriber input1_sub= n.subscribe("/input1", 10, input1_callback); 

	ros::spin();
  return 0;
}
