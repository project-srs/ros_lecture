#include <pluginlib/class_loader.h>
#include <ros_lecture_base/adv_calc_base.h>
#include <string>

int main(int argc, char** argv){
  pluginlib::ClassLoader<ros_lecture_base::calc_base> calc_loader("adv_lecture", "ros_lecture_base::calc_base");

  std::string plugin_name = "plugin_lecture/add";
  try{
    boost::shared_ptr<ros_lecture_base::calc_base> add = calc_loader.createInstance(plugin_name);
    ROS_INFO("%s(2,3)=%i", plugin_name.c_str(), add->op(2,3));
  }
  catch(pluginlib::PluginlibException& ex)
  {
    ROS_ERROR("failed to load add plugin. Error: %s", ex.what());
  }
  return 0;
}

