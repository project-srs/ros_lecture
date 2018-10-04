#include "ros/ros.h"
#include "std_msgs/Empty.h"

int main(int argc, char **argv){
  ros::init(argc, argv, "adv_time");
  ros::NodeHandle n;
  ros::Publisher tick_pub = n.advertise<std_msgs::Empty>("time_tick", 1000);

	ros::spinOnce();
  ros::Duration(1.0).sleep();
  ros::Time ros_begin  = ros::Time::now();
  ros::WallTime wall_begin = ros::WallTime::now();
  
  ros::Rate loop_rate(1);
  while(ros::ok()){
    std_msgs::Empty msg;
    tick_pub.publish(msg);

    ros::Duration ros_duration  = ros::Time::now()-ros_begin;
    ros::WallDuration wall_duration = ros::WallTime::now()-wall_begin;
    ROS_INFO("TICK");
    ROS_INFO("ROS: %u.%u",ros_duration.sec,ros_duration.nsec);
    ROS_INFO("WALL:%u.%u",wall_duration.sec,wall_duration.nsec);

		ros::spinOnce();
		loop_rate.sleep();
  }
  return 0;
}

