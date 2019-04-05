#include <geometry_msgs/PointStamped.h>
#include <ros/ros.h>

geometry_msgs::PointStamped input2_last;
void input2_callback(const geometry_msgs::PointStamped& point_msg) {
  input2_last = point_msg;
}

ros::Publisher output_pub;
void input1_callback(const geometry_msgs::PointStamped& point_msg) {
  geometry_msgs::PointStamped point_data;
  point_data.point.x = point_msg.point.x + input2_last.point.x;
  output_pub.publish(point_data);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "adv_nodetype_semi_event_driven");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  output_pub = nh.advertise<geometry_msgs::PointStamped>("output", 10);
  ros::Subscriber input1_sub = nh.subscribe("input1", 10, input1_callback);
  ros::Subscriber input2_sub = nh.subscribe("input2", 10, input2_callback);

  ros::spin();
  return 0;
}
