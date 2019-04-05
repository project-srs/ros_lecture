#include <geometry_msgs/PointStamped.h>
#include <ros/ros.h>

geometry_msgs::PointStamped input1_last;
void input1_callback(const geometry_msgs::PointStamped& point_msg) {
  input1_last = point_msg;
}

geometry_msgs::PointStamped input2_last;
void input2_callback(const geometry_msgs::PointStamped& point_msg) {
  input2_last = point_msg;
}

float hz = 10;
int main(int argc, char** argv) {
  ros::init(argc, argv, "adv_nodetype_timerdriven");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ros::Publisher output_pub =
      nh.advertise<geometry_msgs::PointStamped>("output", 10);
  ros::Subscriber input1_sub = nh.subscribe("input1", 10, input1_callback);
  ros::Subscriber input2_sub = nh.subscribe("input2", 10, input2_callback);
  pnh.getParam("hz", hz);

  ros::Rate loop_rate(hz);
  while (ros::ok()) {
    geometry_msgs::PointStamped point_msg;

    point_msg.point.x = input1_last.point.x + input2_last.point.x;
    output_pub.publish(point_msg);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
