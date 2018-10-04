#include "ros/ros.h"
  
#include "math.h"
#include "tf/transform_broadcaster.h"

#include <string>

int main(int argc, char **argv){
	ros::init(argc, argv, "vis_tf_publisher");
	ros::NodeHandle n;

	ros::Rate loop_rate(10); 
    int count=0;
	while (ros::ok()){
        float xyz[3]={0,0,0};
        xyz[0]=cos((float)count/5.0);
        xyz[1]=sin((float)count/5.0);
        float rpy[3]={0,0,0};
        std::string source_frame="world";
        std::string target_frame="base_link";

    	static tf::TransformBroadcaster br;
	    tf::Transform transform;
	    transform.setOrigin( tf::Vector3(xyz[0], xyz[1] ,xyz[2]) );
	    tf::Quaternion q;
	    q.setRPY(rpy[0], rpy[1], rpy[2]);
	    transform.setRotation(q);
	    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), source_frame, target_frame));

        count++;
		ros::spinOnce();
		loop_rate.sleep();
	} 
 	return 0;
}

