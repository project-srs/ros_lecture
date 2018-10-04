#include "ros/ros.h"
  
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/JointState.h"

#include "math.h"
#include <sstream>
#include <string>

std::string joint0_name="joint0";
std::string joint1_name="joint1";
std::string joint2_name="joint2";

float wheel_odometry[3]={0};
void odometry0_callback(const std_msgs::Float32& float_msg){
	wheel_odometry[0]=float_msg.data;
}
void odometry1_callback(const std_msgs::Float32& float_msg){
	wheel_odometry[1]=float_msg.data;
}
void odometry2_callback(const std_msgs::Float32& float_msg){
	wheel_odometry[2]=float_msg.data;
}

float wheel_base=0.100;
float wheel_radius=0.20;
void wheel_invert(float *out, float *in){
	float wheel_base=0.0972;
	float wheel_radius=0.019;
	float lv=1.0/wheel_radius;
	float av=wheel_base/wheel_radius;
	float r3=sqrt(3);
	out[0]=+(1.0/r3/lv)*in[0] +0.0       *in[1] -(1.0/r3/lv)*in[2];
	out[1]=-(1.0/3/lv) *in[0] +(2.0/3/lv)*in[1] -(1.0/3/lv) *in[2];
	out[2]=-(1.0/3/av) *in[0] -(1.0/3/av)*in[1] -(1.0/3/av) *in[2];
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "omni_driver");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	//publish
	ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1000);
	ros::Publisher odm_pub = n.advertise<geometry_msgs::Twist>("odm_vel", 1000);

	//Subscribe
	ros::Subscriber odometry0 = n.subscribe("odometry0", 10, odometry0_callback); 
	ros::Subscriber odometry1 = n.subscribe("odometry1", 10, odometry1_callback); 
	ros::Subscriber odometry2 = n.subscribe("odometry2", 10, odometry2_callback); 

	pn.getParam("wheel_base",    wheel_base);
	pn.getParam("wheel_radius",  wheel_radius);
	pn.getParam("joint0_name",  joint0_name);
	pn.getParam("joint1_name",  joint1_name);
	pn.getParam("joint2_name",  joint2_name);

	float dt=1.0/20;
	ros::Rate loop_rate(20); 
	while (ros::ok()){
		//publish joint states
		sensor_msgs::JointState js0;
		js0.header.stamp = ros::Time::now();
		js0.name.resize(3);
		js0.name[0]=joint0_name;
		js0.name[1]=joint1_name;
		js0.name[2]=joint2_name;
		js0.position.resize(3);
		js0.position[0]=wheel_odometry[0];
		js0.position[1]=wheel_odometry[1];
		js0.position[2]=wheel_odometry[2];
		joint_pub.publish(js0);

		//publish odm_vel
		float roll[3]={0};
		static float wheel_odometry_last[3]={0};
		for(int i=0;i<3;i++){
			roll[i]=wheel_odometry[i]-wheel_odometry_last[i];
			wheel_odometry_last[i]=wheel_odometry[i];
		}
		float move[3]={0};
		wheel_invert(move,roll);
		geometry_msgs::Twist odm_msg;
		odm_msg.linear.x=move[0]/dt;
		odm_msg.linear.y=move[1]/dt;
		odm_msg.angular.z=move[2]/dt;
		odm_pub.publish(odm_msg);
		
		ros::spinOnce();
		loop_rate.sleep();
	} 
 	return 0;
}

