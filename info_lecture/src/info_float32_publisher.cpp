#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <math.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "info_float32_publisher");
  ros::NodeHandle nh;
  ros::Publisher float_pub = nh.advertise<std_msgs::Float32>("float32", 10);

  ros::Rate loop_rate(10);
  int count = 0;
  while (ros::ok())
  {
    std_msgs::Float32 float_data;
    float_data.data = sin(0.02 * count * 2 * M_PI);
    float_pub.publish(float_data);

    count++;
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
