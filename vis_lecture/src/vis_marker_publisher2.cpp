#include "ros/ros.h"
#include "math.h"
#include "geometry_msgs/Twist.h"
#include "visualization_msgs/MarkerArray.h"
#include <string>

geometry_msgs::Twist twist_last;
void twist_callback(const geometry_msgs::Twist& twist_msg){
	twist_last=twist_msg;
}

int main(int argc, char **argv){
	ros::init(argc, argv, "vis_marker_publisher2");
	ros::NodeHandle n;

    //publisher
    ros::Publisher marker_pub   = n.advertise<visualization_msgs::MarkerArray>("marker_array", 1);

	//subscriber
	ros::Subscriber twist_sub   = n.subscribe("/cmd_vel", 10, twist_callback); 

	ros::Rate loop_rate(10); 
	while (ros::ok()){
		visualization_msgs::MarkerArray marker_array;
		marker_array.markers.resize(2);

		//marker0
		float length=sqrt(pow(twist_last.linear.x,2)+pow(twist_last.linear.y,2));
		float angle =atan2(twist_last.linear.y,twist_last.linear.x);

		marker_array.markers[0].header.frame_id = "/base_link";
		marker_array.markers[0].header.stamp = ros::Time::now();
		marker_array.markers[0].ns = "basic_shapes";
		marker_array.markers[0].id = 0;
		marker_array.markers[0].lifetime = ros::Duration();
		
		marker_array.markers[0].type = visualization_msgs::Marker::ARROW;
		marker_array.markers[0].action = visualization_msgs::Marker::ADD;
		
		marker_array.markers[0].scale.x = 0.02;
		marker_array.markers[0].scale.y = 0.04;
		marker_array.markers[0].scale.z = 0.1;

		marker_array.markers[0].points.resize(2);
		marker_array.markers[0].points[0].x=0.3*cos(angle);
		marker_array.markers[0].points[0].y=0.3*sin(angle);
		marker_array.markers[0].points[0].z=0;
		marker_array.markers[0].points[1].x=(0.3+length)*cos(angle);
		marker_array.markers[0].points[1].y=(0.3+length)*sin(angle);
		marker_array.markers[0].points[1].z=0;

		marker_array.markers[0].color.r = 0.0f;
		marker_array.markers[0].color.g = 1.0f;
		marker_array.markers[0].color.b = 0.0f;
		marker_array.markers[0].color.a = 1.0f;

		//marker1
		marker_array.markers[1].header.frame_id = "/base_link";
		marker_array.markers[1].header.stamp = ros::Time::now();
		marker_array.markers[1].ns = "basic_shapes";
		marker_array.markers[1].id = 1;
		marker_array.markers[1].lifetime = ros::Duration();
		
		marker_array.markers[1].type = visualization_msgs::Marker::ARROW;
		marker_array.markers[1].action = visualization_msgs::Marker::ADD;
		
		marker_array.markers[1].scale.x = 0.02;
		marker_array.markers[1].scale.y = 0.04;
		marker_array.markers[1].scale.z = 0.1;

		marker_array.markers[1].points.resize(2);
		marker_array.markers[1].points[0].x=0.3;
		marker_array.markers[1].points[0].y=0;
		marker_array.markers[1].points[0].z=0;
		marker_array.markers[1].points[1].x=0.3;
		marker_array.markers[1].points[1].y=0+0.3*twist_last.angular.z;
		marker_array.markers[1].points[1].z=0;

		marker_array.markers[1].color.r = 1.0f;
		marker_array.markers[1].color.g = 0.0f;
		marker_array.markers[1].color.b = 0.0f;
		marker_array.markers[1].color.a = 1.0f;
		
		marker_pub.publish(marker_array);

		ros::spinOnce();
		loop_rate.sleep();
	} 
 	return 0;
}
