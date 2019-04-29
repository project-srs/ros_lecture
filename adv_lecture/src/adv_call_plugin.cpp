#include <pluginlib/class_loader.h>
#include <adv_lecture/adv_calc_base.h>
#include <string>

int main(int argc, char** argv){
  pluginlib::ClassLoader<adv_lecture::calc_base> calc_loader("adv_lecture", "calc_base");

  std::string plugin_name = "plugin_lecture/add";
  try{
    boost::shared_ptr<adv_lecture::calc_base> add = calc_loader.createInstance(plugin_name);
    ROS_INFO("%s(2,3)=%i", plugin_name.c_str(), add->op(2,3));
  }
  catch(pluginlib::PluginlibException& ex)
  {
    ROS_ERROR("failed to load add plugin. Error: %s", ex.what());
  }
  return 0;
}

