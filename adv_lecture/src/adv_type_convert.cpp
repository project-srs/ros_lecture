#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <string>
#include <cmath>


geometry_msgs::Pose setPose(float x, float y, float yaw){
	geometry_msgs::Pose output;
  output.position.x = x;
  output.position.y = y;
  output.position.z = 0;
  output.orientation.x = 0;
  output.orientation.y = 0;
  output.orientation.z = sin(yaw / 2);
  output.orientation.w = cos(yaw / 2);
  return output;
}

void showPose(geometry_msgs::Pose msg){
  printf("pos(x,y,z):   %f, %f, %f\n", msg.position.x, msg.position.y, msg.position.z);
  printf("ori(x,y,z,w): %f, %f, %f %f\n", msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w);
}

tf2::Transform setTF(float x, float y, float yaw){
	tf2::Transform output;
  tf2::Vector3 position(x, y, 0);
  tf2::Quaternion orientation;
  //tf2::Quaternion orientation(qx, qy, qz, qw);
  orientation.setRPY(0, 0, yaw);
              
  output.setOrigin(position);
  output.setRotation(orientation);
  return output;
}

void showTF(tf2::Transform tf){
  double lx=tf.getOrigin().x();
  double ly=tf.getOrigin().y();
  double lz=tf.getOrigin().z();

  double scale = sqrt(1 - tf.getRotation().getW() * tf.getRotation().getW());
  double rx=tf.getRotation().getAxis().x() * scale;
  double ry=tf.getRotation().getAxis().y() * scale;
  double rz=tf.getRotation().getAxis().z() * scale;
  double rw=tf.getRotation().getW();
  printf("pos(x,y,z):   %f, %f, %f\n", lx, ly, lz);
  printf("ori(x,y,z,w): %f, %f, %f %f\n", rx, ry, rz, rw);
}

tf2::Transform convertPoseToTF(geometry_msgs::Pose msg){
  tf2::Transform tf_data;
  tf2::convert(msg, tf_data);
  return tf_data;
}

geometry_msgs::Pose convertTFToPose(tf2::Transform tf_data){
  geometry_msgs::Pose pose;
  tf2::convert(tf_data.getRotation(), pose.orientation);
  pose.position.x = tf_data.getOrigin().getX();
  pose.position.y = tf_data.getOrigin().getY();
  pose.position.z = tf_data.getOrigin().getZ();
  return pose;
}

int main(int argc, char **argv){
	//set & show Pose
  printf("Pose\n");
  geometry_msgs::Pose pose = setPose(2, 3, 1.5707);
  showPose(pose);
  //set & show TF
  printf("TF\n");
  tf2::Transform tf_data = setTF(1, 4, -1.5707);
  showTF(tf_data);
  //convert pose -> tf
  printf("Pose -> TF\n");
  tf2::Transform tf_data2 = convertPoseToTF(pose);
  showTF(tf_data2);
  printf("TF -> Pose\n");
  geometry_msgs::Pose pose2 = convertTFToPose(tf_data);
  showPose(pose2);

 	return 0;
}

