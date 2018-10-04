#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"

int main(int argc, char **argv){
	ros::init(argc, argv, "basic_array_talker");
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<std_msgs::Float32MultiArray>("array", 100);
	ros::Rate loop_rate(1);

	while (ros::ok()){
		std_msgs::Float32MultiArray array;
		array.data.resize(4);
		array.data[0]=0.0;
		array.data[1]=1.0;
		array.data[2]=2.0;
		array.data[3]=3.0;
		pub.publish(array);
		ROS_INFO("I published array!");
		ros::spinOnce();
		loop_rate.sleep();
	}
}
