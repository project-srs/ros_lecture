#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/String.h>

namespace plugin_lecture{
class plugin_nodelet_listener : public nodelet::Nodelet{
public:
  virtual void onInit();
  void chatter_callback(const std_msgs::String& string_msg);
private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
};

void plugin_nodelet_listener::onInit(){
  NODELET_INFO("Listener Init");
  nh_ = getNodeHandle();
  sub_ = nh_.subscribe("chatter", 10, &plugin_nodelet_listener::chatter_callback, this);
}

void plugin_nodelet_listener::chatter_callback(const std_msgs::String& string_msg){
  NODELET_INFO("recieve: %s", string_msg.data.c_str());
}
} // namespace plugin_lecture
PLUGINLIB_EXPORT_CLASS(plugin_lecture::plugin_nodelet_listener, nodelet::Nodelet)
