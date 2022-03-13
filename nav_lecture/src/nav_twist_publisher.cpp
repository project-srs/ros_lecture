#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Joy.h>

class TwistPublisher{
public:
  TwistPublisher() : nh_(), pnh_("~") {
    cmd_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("cmd_vel", 1);
    activate_pub_ = nh_.advertise<std_msgs::Bool>("activate", 1);
    joy_sub_ = nh_.subscribe("joy", 10, &TwistPublisher::joyCallback, this);
    timer_ = nh_.createTimer(ros::Duration(0.1), &TwistPublisher::timerCallback, this);
  }

  void joyCallback(const sensor_msgs::Joy& joy_msg) {
    last_joy_ = joy_msg;
  }

  void timerCallback(const ros::TimerEvent& e) {
    geometry_msgs::TwistStamped cmd_vel;
    cmd_vel.header.frame_id = "base_link";
    cmd_vel.header.stamp = ros::Time::now();
    cmd_vel.twist = generateTwist(last_joy_);
    cmd_pub_.publish(cmd_vel);

    std_msgs::Bool bool_msg;
    int activate = generateActivate(last_joy_);
    if(0 < activate){
      bool_msg.data = true;
      activate_pub_.publish(bool_msg);
    } else if(activate < 0){
      bool_msg.data = false;
      activate_pub_.publish(bool_msg);
    }
  }

private:
  geometry_msgs::Twist generateTwist(const sensor_msgs::Joy joy) const {
    int assign_x = 1;
    int assign_y = 0;
    int assign_z = 3;
    pnh_.getParamCached("assign_x", assign_x);
    pnh_.getParamCached("assign_y", assign_y);
    pnh_.getParamCached("assign_z", assign_z);

    float max_x = 0.5;
    float max_y = 0.5;
    float max_z = 1.5;
    pnh_.getParamCached("max_x", max_x);
    pnh_.getParamCached("max_y", max_y);
    pnh_.getParamCached("max_z", max_z);

    geometry_msgs::Twist twist;
    if(0 <= assign_x && assign_x < joy.axes.size()){
      twist.linear.x = max_x * joy.axes[assign_x];
    }
    if(0 <= assign_y && assign_y < joy.axes.size()){
      twist.linear.y = max_y * joy.axes[assign_y];
    }
    if(0 <= assign_z && assign_z < joy.axes.size()){
      twist.angular.z = max_z * joy.axes[assign_z];
    }
    return twist;
  }

  int generateActivate(const sensor_msgs::Joy joy) {
    int output = 0;

    int assign_activate;
    pnh_.getParamCached("assign_activate", assign_activate);
    if(0 <= assign_activate && assign_activate < joy.buttons.size()) {
      int activate_button = joy.buttons[assign_activate];
      if(activate_button < 0 && 0 < last_active_button_) {
        output = -1;
      } else if(0 < activate_button && last_active_button_ < 0) {
        output = 1;
      }
      last_active_button_ = activate_button;
    }
    return output;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher cmd_pub_;
  ros::Publisher activate_pub_;
  ros::Subscriber joy_sub_;
  ros::Timer timer_;
  sensor_msgs::Joy last_joy_;
  int last_active_button_{0}; // initial value
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "basic_twist_publisher");
  TwistPublisher twist_publisher;
  ros::spin();
  return 0;
}
