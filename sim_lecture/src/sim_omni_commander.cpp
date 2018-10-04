#include "ros/ros.h"
  
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"

#include "math.h"
#include <sstream>
#include <string>


geometry_msgs::Twist twist_last;
bool twist_enable;
void twist_stamped_callback(const geometry_msgs::Twist& twist_msg){
	twist_last=twist_msg;
	twist_enable=true;
}

float wheel_base=0.100;
float wheel_radius=0.20;
float wheel[3]={M_PI/3, M_PI, 5*M_PI/3};
float wheel_normal[3];

void calculation(float *out, float *in){
	for(int i=0;i<3;i++){
		out[i]=(cos(wheel_normal[i])*in[0]+sin(wheel_normal[i])*in[1])/wheel_radius;
		out[i]+=-in[2]*wheel_base/wheel_radius;
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "omni_driver");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	//rosparam
	pn.getParam("wheel_base",    wheel_base);
	pn.getParam("wheel_radius",  wheel_radius);
	pn.getParam("wheel0", wheel[0]);
	pn.getParam("wheel1", wheel[1]);
	pn.getParam("wheel2", wheel[2]);

	//publish
	ros::Publisher wheel0_pub = n.advertise<std_msgs::Float64>("wheel0", 10);
	ros::Publisher wheel1_pub = n.advertise<std_msgs::Float64>("wheel1", 10);
	ros::Publisher wheel2_pub = n.advertise<std_msgs::Float64>("wheel2", 10);
	//Subscribe
	ros::Subscriber joy_sub     = n.subscribe("cmd_vel", 10, twist_stamped_callback); 

	for(int i=0;i<3;i++)wheel_normal[i]=wheel[i]-M_PI/2;
	
	ros::Rate loop_rate(20); 
	while (ros::ok()){
		if(twist_enable){
			float in[3]={0};
			float out[3]={0};
			in[0]=twist_last.linear.x;
			in[1]=twist_last.linear.y;
			in[2]=twist_last.angular.z;
			calculation(out,in);
			std_msgs::Float64 data[3];
			data[0].data=out[0];
			data[1].data=out[1];
			data[2].data=out[2];
			wheel0_pub.publish(data[0]);
			wheel1_pub.publish(data[1]);
			wheel2_pub.publish(data[2]);
		}
		
		ros::spinOnce();
		loop_rate.sleep();
	} 
 	return 0;
}

