#include <ros/ros.h>
#include <ros/package.h>

int main(int argc, char** argv)
{
  ROS_INFO("get path");
  std::vector<std::string> model_paths;
  ros::package::getPlugins("gazebo_ros","gazebo_model_path",model_paths);
  for (std::vector<std::string>::iterator iter=model_paths.begin(); iter != model_paths.end(); iter++)
  {
    ROS_INFO("Model path %s",(*iter).c_str());
  }
}