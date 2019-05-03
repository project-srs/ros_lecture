#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/String.h>

namespace plugin_lecture{
class plugin_nodelet_talker : public nodelet::Nodelet{
public:
  virtual void onInit();
  void timer_callback(const ros::TimerEvent&);
private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher pub_;
  ros::Timer timer_;
  std::string content_;
};

void plugin_nodelet_talker::onInit(){
  NODELET_INFO("Talker Init");
  nh_ = getNodeHandle();
  pnh_ = getPrivateNodeHandle();

  content_ = "hello";
  pnh_.getParam("content", content_);
  pub_ = nh_.advertise<std_msgs::String>("chatter", 10);
  timer_ = nh_.createTimer(ros::Duration(1.0), &plugin_nodelet_talker::timer_callback, this);
}

void plugin_nodelet_talker::timer_callback(const ros::TimerEvent&){
  NODELET_INFO("send: %s", content_.c_str());
  std_msgs::String string_msg;
  string_msg.data = content_;
  pub_.publish(string_msg);
}
} // namespace plugin_lecture
PLUGINLIB_EXPORT_CLASS(plugin_lecture::plugin_nodelet_talker, nodelet::Nodelet);
