#include <ros/ros.h>

#include <control_msgs/JointControllerState.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#include <string>
#include <math.h>

float wheel_speed[3]={0};
void state0_callback(const control_msgs::JointControllerState& state_msg){
  wheel_speed[0]=state_msg.process_value;
}
void state1_callback(const control_msgs::JointControllerState& state_msg){
  wheel_speed[1]=state_msg.process_value;
}
void state2_callback(const control_msgs::JointControllerState& state_msg){
  wheel_speed[2]=state_msg.process_value;
}

float wheel_base=0.100;
float wheel_radius=0.20;
float publish_rate=20;
std::string frame_id="odom";
geometry_msgs::Twist wheel_invert(float *in){
  geometry_msgs::Twist out;
  float lv=1.0/wheel_radius;
  float av=wheel_base/wheel_radius;
  float r3=sqrt(3);
  out.linear.x  = +(1.0/r3/lv)*in[0] +0.0       *in[1] -(1.0/r3/lv)*in[2];
  out.linear.y  = -(1.0/3/lv) *in[0] +(2.0/3/lv)*in[1] -(1.0/3/lv) *in[2];
  out.angular.z = -(1.0/3/av) *in[0] -(1.0/3/av)*in[1] -(1.0/3/av) *in[2];
  return out;
}
geometry_msgs::Pose update_pose(geometry_msgs::Pose last_pose, geometry_msgs::Twist speed, float dt){
  geometry_msgs::Pose next_pose;

  tf::Quaternion quat_tmp;
  double roll, pitch, last_yaw=0;
  //get yaw
  quaternionMsgToTF(last_pose.orientation, quat_tmp);
  tf::Matrix3x3(quat_tmp).getRPY(roll, pitch, last_yaw); 
  //update
  next_pose.position.x = last_pose.position.x + (cos(last_yaw+speed.angular.z * dt/2) * speed.linear.x - sin(last_yaw+speed.angular.z * dt/2) * speed.linear.y) * dt;
  next_pose.position.y = last_pose.position.y + (sin(last_yaw+speed.angular.z * dt/2) * speed.linear.x + cos(last_yaw+speed.angular.z * dt/2) * speed.linear.y) * dt;
  float next_yaw = last_yaw + speed.angular.z * dt;
  //sey yaw
  quat_tmp=tf::createQuaternionFromRPY(0,0,next_yaw);
  quaternionTFToMsg(quat_tmp, next_pose.orientation);
  return next_pose;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "s4_omni_odom");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  //publish
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 10);
  //Subscribe
  ros::Subscriber odometry0 = n.subscribe("wheel0/state", 10, state0_callback); 
  ros::Subscriber odometry1 = n.subscribe("wheel1/state", 10, state1_callback); 
  ros::Subscriber odometry2 = n.subscribe("wheel2/state", 10, state2_callback); 

  pn.getParam("wheel_base",    wheel_base);
  pn.getParam("wheel_radius",  wheel_radius);
  pn.getParam("publish_rate", publish_rate);
  pn.getParam("frame_id", frame_id);

  float dt=1.0/publish_rate;
  ros::Rate loop_rate(publish_rate);
  geometry_msgs::Pose body_position;
  body_position.orientation.w=1.0;
  while (ros::ok()){
    geometry_msgs::Twist body_speed;
    body_speed=wheel_invert(wheel_speed);

    body_position=update_pose(body_position, body_speed, dt);

    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp=ros::Time::now();
    odom_msg.header.frame_id=frame_id;
    odom_msg.twist.twist=body_speed;
    odom_msg.pose.pose=body_position;
    odom_pub.publish(odom_msg);

    ros::spinOnce();
    loop_rate.sleep();
  } 
  return 0;
}
