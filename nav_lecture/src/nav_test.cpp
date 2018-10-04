#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"
#include "gazebo_msgs/ModelStates.h"
#include "tf/transform_broadcaster.h"

#include "math.h"
#include <string>
#include <iostream>
#include <sstream>
#include <random>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "nav_model_base_publisher");
	ros::NodeHandle n;

	ros::Rate loop_rate(2); 
	while (ros::ok()){
		ros::Time time0=ros::Time::now();
		ROS_INFO("%u.%u",(int)(time0.sec),(int)(time0.nsec));
		ros::spinOnce();
		loop_rate.sleep();
	} 
 	return 0;
}
