#include "ros/ros.h"

#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <geographic_msgs/GeoPointStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/HomePosition.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>

class MavrosBridge
{
public:
  MavrosBridge() : nh_(), pnh_("~")
  {
    joy_pub_ = nh_.advertise<sensor_msgs::Joy>("/mavros_bridge/joy", 10);
    set_gp_origin_pub_ = nh_.advertise<geographic_msgs::GeoPointStamped>("/mavros/global_position/set_gp_origin", 10);
    mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    arm_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    rc_sub_ = nh_.subscribe("/mavros/rc/in", 1, &MavrosBridge::rcCallback, this);
    activate_sub_ = nh_.subscribe("/mavros_bridge/activate", 1, &MavrosBridge::activateCallback, this);
    state_sub_ = nh_.subscribe("/mavros/state", 1, &MavrosBridge::stateCallback, this);
    hp_sub_ = nh_.subscribe("/mavros/home_position/home", 1, &MavrosBridge::hpCallback, this);
  }

  void rcCallback(mavros_msgs::RCIn msg)
  {
    sensor_msgs::Joy joy = convertRCtoJoy(msg);
    joy_pub_.publish(joy);
  }

  void activateCallback(std_msgs::Bool msg)
  {
    ROS_INFO("activate %u", msg.data);
    activate(msg.data);
  }

  void stateCallback(mavros_msgs::State msg)
  {
    last_state_ = msg;

    if (!initialized_)
    {
      initialSetup();
      initialized_ = true;
    }
  }

  void hpCallback(mavros_msgs::HomePosition msg)
  {
    hp_valid_ = true;
  }

  bool checkMove(void)
  {
    bool guided = last_state_.mode == "GUIDED";
    return last_state_.armed && guided && hp_valid_;
  }

  sensor_msgs::Joy convertRCtoJoy(const mavros_msgs::RCIn& msg)
  {
    sensor_msgs::Joy joy;
    joy.axes.resize(4);
    joy.buttons.resize(1);
    if (5 <= msg.channels.size())
    {
      float x = -((float)msg.channels[2] - 1510) / 410;
      float y = -((float)msg.channels[0] - 1510) / 410;
      float z = -((float)msg.channels[1] - 1510) / 410;
      float r = -((float)msg.channels[3] - 1510) / 410;
      joy.axes[0] = x;
      joy.axes[1] = y;
      joy.axes[2] = z;
      joy.axes[3] = r;

      auto getButton = [](const int b) {
        int output = 0;
        if (b < 1300)
          output = -1;
        else if (b < 1700)
          output = 0;
        else
          output = 1;
        return output;
      };
      joy.buttons[0] = getButton(msg.channels[4]);
    }
    return joy;
  }

  void initialSetup(void)
  {
    ROS_INFO("set GP origin");
    geographic_msgs::GeoPointStamped geo;
    set_gp_origin_pub_.publish(geo);
  }

  bool activate(bool activate)
  {
    ROS_INFO("set GUIDED");
    mavros_msgs::SetMode mode;
    mode.request.base_mode = 0;
    mode.request.custom_mode = "GUIDED";
    mode_client_.call(mode);

    ROS_INFO("set %s", activate ? "arm" : "disarm");
    mavros_msgs::CommandBool cmd;
    cmd.request.value = activate;
    arm_client_.call(cmd);

    return true;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher joy_pub_;
  ros::Subscriber rc_sub_;
  ros::Subscriber activate_sub_;
  ros::Subscriber state_sub_;
  ros::Subscriber hp_sub_;

  // for initial setup
  ros::Publisher set_gp_origin_pub_;
  ros::ServiceClient mode_client_;
  ros::ServiceClient arm_client_;

  mavros_msgs::State last_state_;
  bool hp_valid_{ false };
  bool initialized_{ false };
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mavros_bridge");
  MavrosBridge mavros_bridge;
  ros::spin();
  return 0;
}
