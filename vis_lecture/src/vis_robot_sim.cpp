#include "ros/ros.h"
  
#include "math.h"  
#include <stdlib.h>
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Joy.h"

#include <tf/transform_broadcaster.h>

#include <string>
#include <iostream>
#include <sstream>

void robot_pose_publish(float *position){
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin( tf::Vector3(position[0], position[1], 0.0) );
	tf::Quaternion q;
	q.setRPY(0, 0, position[2]);
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "base_link"));
}

float position[3]={0};//x,y,zr
float velocity[3]={0};//x,y,zr
void robot_tick(geometry_msgs::Twist cmd_vel, float dt){
	velocity[0]=cmd_vel.linear.x;
	velocity[1]=cmd_vel.linear.y;
	velocity[2]=cmd_vel.angular.z;

	position[0]+=(cos(position[2])*velocity[0]-sin(position[2])*velocity[1])*dt;
	position[1]+=(sin(position[2])*velocity[0]+cos(position[2])*velocity[1])*dt;
	position[2]+=velocity[2]*dt;
}

geometry_msgs::Twist twist_last;
void twist_callback(const geometry_msgs::Twist& twist_msg){
	twist_last=twist_msg;
}

float normal_rad(float value){
	if(value>0)return fmod(value,3.1415);
	else return -fmod(-value,3.1415);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "vis_robot_sim");
	ros::NodeHandle n;
	
	ros::Subscriber twist_sub= n.subscribe("/cmd_vel", 10, twist_callback); 
	
	srand((unsigned int)time(NULL));

	ros::Rate loop_rate(20); 
	while (ros::ok()){
		robot_tick(twist_last, 0.05);
		robot_pose_publish(position);
		ros::spinOnce();
		loop_rate.sleep();
	} 
 	return 0;
}

