#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/transform_broadcaster.h>

#include <string>
#include <math.h>

void robot_pose_publish(geometry_msgs::Pose2D pose)
{
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(pose.x, pose.y, 0.0));
  tf::Quaternion q;
  q.setRPY(0, 0, pose.theta);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "base_link"));
}

geometry_msgs::Pose2D robot_tick(geometry_msgs::Pose2D pose, geometry_msgs::Twist cmd_vel, float dt)
{
  geometry_msgs::Pose2D output;
  float tmp_rz = pose.theta + cmd_vel.angular.z * dt / 2;
  output.x = pose.x + (cos(tmp_rz) * cmd_vel.linear.x - sin(tmp_rz) * cmd_vel.linear.y) * dt;
  output.y = pose.y + (sin(tmp_rz) * cmd_vel.linear.x + cos(tmp_rz) * cmd_vel.linear.y) * dt;
  output.theta = pose.theta + cmd_vel.angular.z * dt;
  return output;
}

geometry_msgs::Twist twist_last;
void twist_callback(const geometry_msgs::Twist& twist_msg)
{
  twist_last = twist_msg;
}

float normal_rad(float value)
{
  if (value > 0)
    return fmod(value, 3.1415);
  else
    return -fmod(-value, 3.1415);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vis_robot_sim");
  ros::NodeHandle nh;
  ros::Subscriber twist_sub = nh.subscribe("/cmd_vel", 10, twist_callback);
  geometry_msgs::Pose2D pose;

  ros::Rate loop_rate(20);
  while (ros::ok())
  {
    pose = robot_tick(pose, twist_last, 0.05);
    robot_pose_publish(pose);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
