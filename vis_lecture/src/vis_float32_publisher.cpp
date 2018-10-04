#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "math.h"

int main(int argc, char **argv){
	ros::init(argc, argv, "vis_float32_publisher");
	ros::NodeHandle n;

	//publisher
	ros::Publisher float_pub = n.advertise<std_msgs::Float32>("float32", 10);

	ros::Rate loop_rate(10);
	int count=0;
	while (ros::ok()){
		//publish joint states
		std_msgs::Float32 float_data;
		float_data.data=sin(count*3.14/10/2);
		float_pub.publish(float_data);
		
		count++;
		ros::spinOnce();
		loop_rate.sleep();
	} 
 	return 0;
}

