#include "ros/ros.h"
#include "basic_lecture/cmsg.h"

int main(int argc, char **argv){
	ros::init(argc, argv, "basic_cmsg_talker");
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<basic_lecture::cmsg>("chatter", 100);
	ros::Rate loop_rate(1);

	while (ros::ok()){
		basic_lecture::cmsg data;
		data.word="hello";
		data.number=10;
		pub.publish(data);
		ros::spinOnce();
		loop_rate.sleep();
	}
}
