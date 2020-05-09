#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include <time.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "adv_time");
  ros::NodeHandle n;
  ros::Publisher tick_pub = n.advertise<std_msgs::Empty>("time_tick", 10);

  ros::Time::waitForValid();
  ros::Time ros_begin = ros::Time::now();
  ros::WallTime wall_begin = ros::WallTime::now();

  ros::Rate loop_rate(1);
  while (ros::ok())
  {
    ROS_INFO("TICK");
    std_msgs::Empty msg;
    tick_pub.publish(msg);

    ros::Time ros_now = ros::Time::now();
    ros::Duration ros_duration = ros_now - ros_begin;
    ROS_INFO("ROS: %u.09%u", ros_duration.sec, ros_duration.nsec);

    ros::WallTime wall_now = ros::WallTime::now();
    ros::WallDuration wall_duration = wall_now - wall_begin;
    ROS_INFO("WALL:%u.%09u", wall_duration.sec, wall_duration.nsec);

    char date[64];
    time_t t = ros::Time::now().sec;
    strftime(date, sizeof(date), "%Y/%m/%d %a %H:%M:%S", localtime(&t));
    ROS_INFO("%s", date);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
