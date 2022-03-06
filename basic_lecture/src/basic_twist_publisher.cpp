#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

class TwistPublisher{
public:
  TwistPublisher() : nh_(), pnh_("~") {
    cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    joy_sub_ = nh_.subscribe("joy", 10, &TwistPublisher::joyCallback, this);
    timer_ = nh_.createTimer(ros::Duration(0.1), &TwistPublisher::timerCallback, this);
  }

  void joyCallback(const sensor_msgs::Joy& joy_msg) {
    last_joy_ = joy_msg;
  }

  void timerCallback(const ros::TimerEvent& e) {
    int assign_x = 1;
    int assign_y = 0;
    int assign_z = 3;
    pnh_.getParam("assign_x", assign_x);
    pnh_.getParam("assign_y", assign_y);
    pnh_.getParam("assign_z", assign_z);

    float max_x = 0.5;
    float max_y = 0.5;
    float max_z = 1.5;
    pnh_.getParam("max_x", max_x);
    pnh_.getParam("max_y", max_y);
    pnh_.getParam("max_z", max_z);

    geometry_msgs::Twist cmd_vel;
    if(0 <= assign_x && assign_x < last_joy_.axes.size()){
      cmd_vel.linear.x = max_x * last_joy_.axes[assign_x];
    }
    if(0 <= assign_y && assign_y < last_joy_.axes.size()){
      cmd_vel.linear.y = max_y * last_joy_.axes[assign_y];
    }
    if(0 <= assign_z && assign_z < last_joy_.axes.size()){
      cmd_vel.angular.z = max_z * last_joy_.axes[assign_z];
    }
    cmd_pub_.publish(cmd_vel);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher cmd_pub_;
  ros::Subscriber joy_sub_;
  ros::Timer timer_;
  sensor_msgs::Joy last_joy_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "basic_twist_publisher");
  TwistPublisher twist_publisher;
  ros::spin();
  return 0;
}
