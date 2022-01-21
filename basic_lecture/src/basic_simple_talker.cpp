#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ros/package.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "basic_simple_talker");
  ros::NodeHandle nh;
  ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 10);
  ros::Rate loop_rate(10);

  ROS_INFO("get path");
  std::vector<std::string> model_paths;
  ros::package::getPlugins("gazebo_ros","gazebo_model_path",model_paths);
  for (std::vector<std::string>::iterator iter=model_paths.begin(); iter != model_paths.end(); iter++)
  {
    ROS_INFO("Model path %s",(*iter).c_str());
    // gazebo::common::SystemPaths::Instance()->AddModelPaths(iter->c_str());
  }

  while (ros::ok())
  {
    std_msgs::String msg;
    msg.data = "hello world!";
    ROS_INFO("publish: %s", msg.data.c_str());
    chatter_pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
