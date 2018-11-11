#include "ros/ros.h"
#include "std_msgs/Float32.h"

float input1_last=0;
void input1_callback(const std_msgs::Float32& float_msg){
  input1_last=float_msg.data;
}

float input2_last=0;
void input2_callback(const std_msgs::Float32& float_msg){
  input2_last=float_msg.data;
}

float hz=10;
int main(int argc, char **argv){
  ros::init(argc, argv, "adv_nodetype_sync");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  ros::Publisher output_pub = n.advertise<std_msgs::Float32>("output", 10);
	ros::Subscriber input1_sub= n.subscribe("/input1", 10, input1_callback); 
	ros::Subscriber input2_sub= n.subscribe("/input2", 10, input2_callback); 
  pn.getParam("hz",hz);

  ros::Rate loop_rate(hz);
  while(ros::ok()){
    std_msgs::Float32 float_msg;
    float_msg.data = input1_last + input2_last;
    output_pub.publish(float_msg);

		ros::spinOnce();
		loop_rate.sleep();
  }
  return 0;
}

