#include <geometry_msgs/PointStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <ros/ros.h>

ros::Publisher output_pub;
void sync_callback(const geometry_msgs::PointStamped &point1_msg,
                   const geometry_msgs::PointStamped &point2_msg) {
  geometry_msgs::PointStamped point_msg;
  point_msg.header.stamp = ros::Time::now();
  point_msg.point.x = point1_msg.point.x + point2_msg.point.x;
  output_pub.publish(point_msg);
}
typedef message_filters::sync_policies::ApproximateTime<
    geometry_msgs::PointStamped, geometry_msgs::PointStamped>
    MySyncPolicy;
int main(int argc, char **argv) {
  ros::init(argc, argv, "adv_nodetype_message_filter");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  output_pub = nh.advertise<geometry_msgs::PointStamped>("output", 10);

  message_filters::Subscriber<geometry_msgs::PointStamped> point_sub1(
      nh, "input1", 10);
  message_filters::Subscriber<geometry_msgs::PointStamped> point_sub2(
      nh, "input2", 10);
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), point_sub1,
                                                   point_sub2);
  sync.registerCallback(&sync_callback);

  printf("start\n");
  ros::spin();
  return 0;
}
