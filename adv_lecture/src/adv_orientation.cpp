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

/*
2. quaternion-1: tf::fransform
1. quaternion-2: geometry_msgs::Quaternion
3. rpy:	float[3]
4. rotate vector: geometry_msgs::vector3
5. rotate matrix: geometry_msgs::mat3
*/


void tf_quat_to_rpy(double& roll, double& pitch, double& yaw, tf::Quaternion quat){
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);  //rpy are Pass by Reference
}
void geometry_quat_to_rpy(double& roll, double& pitch, double& yaw, geometry_msgs::Quaternion geometry_quat){
	tf::Quaternion quat;
	quaternionMsgToTF(geometry_quat, quat);
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);  //rpy are Pass by Reference
}
tf::Quaternion rpy_to_tf_quat(double roll, double pitch, double yaw){
	return tf::createQuaternionFromRPY(roll, pitch, yaw);
}
geometry_msgs::Quaternion rpy_to_geometry_quat(double roll, double pitch, double yaw){
	tf::Quaternion quat=tf::createQuaternionFromRPY(roll,pitch,yaw);
	geometry_msgs::Quaternion geometry_quat;
	quaternionTFToMsg(quat, geometry_quat);
	return geometry_quat;
}
int main(int argc, char **argv){
	printf("start\n");
	double roll, pitch, yaw;
	tf::Quaternion quat;
	geometry_msgs::Quaternion geometry_quat;
	
	printf("\ntf_quat to rpy\n");
	quat=tf::Quaternion(0.707,0,0,0.707);
	tf_quat_to_rpy(roll, pitch, yaw, quat);
	printf("input:  tf_Quat x:%f, y:%f, z:%f, w:%f\n", quat.getAxis().x(), quat.getAxis().y(), quat.getAxis().z(), quat.getW());
	printf("output: rpy roll:%f, pitch:%f, yaw:%f\n", roll, pitch, yaw);

	printf("\ngeometry_quat to rpy\n");
	geometry_quat.x=1;
	geometry_quat.y=0;
	geometry_quat.z=0;
	geometry_quat.w=0;
	geometry_quat_to_rpy(roll, pitch, yaw, geometry_quat);
	printf("input:  geoQuat x:%f, y:%f, z:%f, w:%f\n", geometry_quat.x, geometry_quat.y, geometry_quat.z, geometry_quat.w);
	printf("output: rpy roll:%f, pitch:%f, yaw:%f\n", roll, pitch, yaw);

	printf("\nrpy to tf_quat\n");
	roll=0;
	pitch=0;
	yaw=3.141592/2;
	quat=rpy_to_tf_quat(roll, pitch, yaw);
	printf("input:  rpy roll:%f, pitch:%f, yaw:%f\n", roll, pitch, yaw);
	printf("output: tf_Quat x:%f, y:%f, z:%f, w:%f\n", quat.getAxis().x(), quat.getAxis().y(), quat.getAxis().z(), quat.getW());

	printf("\nrpy_to_geometry_quat\n");
	roll=0;
	pitch=0;
	yaw=3.141592/2;
	geometry_quat=rpy_to_geometry_quat(roll, pitch, yaw);
	printf("input:  rpy roll:%f, pitch:%f, yaw:%f\n", roll, pitch, yaw);
	printf("output: geoQuat x:%f, y:%f, z:%f, w:%f\n", geometry_quat.x, geometry_quat.y, geometry_quat.z, geometry_quat.w);

 	return 0;
}

