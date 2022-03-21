#include "ros/ros.h"

#include <sensor_msgs/CameraInfo.h>

class CameraInfoModifier
{
public:
  CameraInfoModifier() : nh_(), pnh_("~")
  {
    camera_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("camera_info_modified", 10);
    camera_info_sub_ = nh_.subscribe("camera_info", 1, &CameraInfoModifier::callback, this);
  }

  void callback(const sensor_msgs::CameraInfo& msg)
  {
    sensor_msgs::CameraInfo output = msg;

    float baseline_x = 0.0f;
    pnh_.getParamCached("baseline_x", baseline_x);

    if(4 <= output.P.size()){
        output.P[3] = - baseline_x * output.P[0];
        ROS_INFO_ONCE("override P[3] %f", output.P[3]);
    }
    camera_info_pub_.publish(output);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher camera_info_pub_;
  ros::Subscriber camera_info_sub_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "camera_info_modifier");
  CameraInfoModifier camera_info_modifier;
  ros::spin();
  return 0;
}
