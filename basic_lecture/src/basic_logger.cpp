#include "ros/ros.h"
#include <string.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "basic_logger");
	ros::NodeHandle n;
	ros::Rate loop_rate(1);
	/*if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
		ros::console::notifyLoggerLevelsChanged();
	}*/

	int count=0;
	while (ros::ok()){
		ROS_DEBUG("log:%i", count);
		ROS_INFO( "log:%i", count);
		ROS_WARN( "log:%i", count);
		ROS_ERROR("log:%i", count);
		ROS_FATAL("log:%i", count);

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
