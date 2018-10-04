#include <pluginlib/class_loader.h>
#include <adv_lecture/adv_calc_base.h>

int main(int argc, char** argv)
{
  pluginlib::ClassLoader<calc_base> calc_loader("adv_lecture", "calc_base");

  try
  {
    boost::shared_ptr<calc_base> add = calc_loader.createInstance("add");
    ROS_INFO("add(2,3)=%i",add->op(2,3));
  }
  catch(pluginlib::PluginlibException& ex)
  {
    ROS_ERROR("failed to load add plugin. Error: %s", ex.what());
  }
  return 0;
}

