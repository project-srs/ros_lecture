#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>

ros::Publisher output_pub;
void input1_callback(const geometry_msgs::PointStamped& point_msg){
  output_pub.publish(point_msg);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "adv_nodetype_event_driven");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  output_pub = nh.advertise<geometry_msgs::PointStamped>("output", 10);
	ros::Subscriber input1_sub= nh.subscribe("input1", 10, input1_callback); 

	ros::spin();
  return 0;
}
