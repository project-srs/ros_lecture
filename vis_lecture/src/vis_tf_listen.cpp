#include "ros/ros.h"
  
#include "math.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_listener.h"

#include <string>

int main(int argc, char **argv){
	ros::init(argc, argv, "vis_tf_listen");
	ros::NodeHandle n;
	
	tf::TransformListener tflistener;

	ros::Rate loop_rate(20); 
	while (ros::ok()){

		geometry_msgs::PoseStamped source_pose;
		source_pose.header.frame_id="link1";
		source_pose.pose.orientation.w=1.0;
		geometry_msgs::PoseStamped target_pose;
		try{
			tflistener.waitForTransform("world", "link1", ros::Time(0), ros::Duration(1.0));
			tflistener.transformPose("world",ros::Time(0),source_pose,"link1",target_pose);
		
            ROS_INFO("x:%03f, y:%03f,z:%03f",target_pose.pose.position.x,target_pose.pose.position.y,target_pose.pose.position.z);
		}
		catch(...){
            ROS_INFO("tf error");
		}
		ros::spinOnce();
		loop_rate.sleep();
	} 
 	return 0;
}

