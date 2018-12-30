#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <string>
#include <math.h>

int main(int argc, char **argv){
  ros::init(argc, argv, "vis_joint_publisher");
  ros::NodeHandle nh;

  //publisher
  ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 10);

  ros::Rate loop_rate(10);
  int count=0;
  while (ros::ok()){
    sensor_msgs::JointState js0;
    js0.header.stamp = ros::Time::now();
    js0.name.resize(2);
    js0.name[0]="body2_joint";
    js0.name[1]="body3_joint";
    js0.position.resize(2);
    js0.position[0]=-1.0*(float)count/40.0;
    js0.position[1]= 2.0*(float)count/40.0;
    joint_pub.publish(js0);
    count++;

    ros::spinOnce();
    loop_rate.sleep();
  } 
  return 0;
}
