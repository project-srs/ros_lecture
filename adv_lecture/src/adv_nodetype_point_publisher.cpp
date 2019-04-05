#include <geometry_msgs/PointStamped.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "adv_nodetype_point_publisher");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ros::Publisher output_pub =
      nh.advertise<geometry_msgs::PointStamped>("output", 10);
  float hz = 10;
  pnh.getParam("hz", hz);
  float offset = 0.0;
  pnh.getParam("offset", offset);

  ros::Duration(offset).sleep();
  ros::Rate loop_rate(hz);
  while (ros::ok()) {
    geometry_msgs::PointStamped point_msg;
    point_msg.header.stamp = ros::Time::now();
    output_pub.publish(point_msg);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
