#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <string>

/*
2. quaternion-1: tf::fransform
1. quaternion-2: geometry_msgs::Quaternion
3. rpy:	float[3]
4. rotate vector: geometry_msgs::vector3
5. rotate matrix: geometry_msgs::mat3
*/

void tf_quat_to_rpy(tf::Quaternion quat, double& roll, double& pitch, double& yaw){
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);  //rpy are Pass by Reference
}
void geometry_quat_to_rpy(geometry_msgs::Quaternion geometry_quat, double& roll, double& pitch, double& yaw){
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

void calc_quat_to_rpy(geometry_msgs::Quaternion geometry_quat, double& roll, double& pitch, double& yaw){
	float q0q0 = geometry_quat.w * geometry_quat.w;
	float q1q1 = geometry_quat.x * geometry_quat.x;
	float q2q2 = geometry_quat.y * geometry_quat.y;
	float q3q3 = geometry_quat.z * geometry_quat.z;
	float q0q1 = geometry_quat.w * geometry_quat.x;
	float q0q2 = geometry_quat.w * geometry_quat.y;
	float q0q3 = geometry_quat.w * geometry_quat.z;
	float q1q2 = geometry_quat.x * geometry_quat.y;
	float q1q3 = geometry_quat.x * geometry_quat.z;
	float q2q3 = geometry_quat.y * geometry_quat.z;

    roll = atan2f((2.f * (q2q3 + q0q1)), (q0q0 - q1q1 - q2q2 + q3q3));
    pitch = -asinf((2.f * (q1q3 - q0q2)));
    yaw = atan2f((2.f * (q1q2 + q0q3)), (q0q0 + q1q1 - q2q2 - q3q3));
}
geometry_msgs::Quaternion calc_rpy_to_quat(double roll, double pitch, double yaw){
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    geometry_msgs::Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
	return q;
}

int main(int argc, char **argv){
	printf("start\n");
	double roll, pitch, yaw;
	tf::Quaternion quat;
	geometry_msgs::Quaternion geometry_quat;
	
	printf("\ntf_quat to rpy\n");
	quat=tf::Quaternion(0.707,0,0,0.707);
	tf_quat_to_rpy(quat, roll, pitch, yaw);
	printf("input:  tf_Quat x:%f, y:%f, z:%f, w:%f\n", quat.getAxis().x(), quat.getAxis().y(), quat.getAxis().z(), quat.getW());
	printf("output: rpy roll:%f, pitch:%f, yaw:%f\n", roll, pitch, yaw);

	printf("\ngeometry_quat to rpy\n");
	geometry_quat.x=1;
	geometry_quat.y=0;
	geometry_quat.z=0;
	geometry_quat.w=0;
	geometry_quat_to_rpy(geometry_quat, roll, pitch, yaw);
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

