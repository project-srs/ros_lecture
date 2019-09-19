#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "moveit_commander");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(2);
  spinner.start();

  // Set up the arm planning interface
  moveit::planning_interface::MoveGroupInterface arm("arm0");

  // Prepare
  ROS_INFO("Moving to prepare pose");
  ROS_INFO_NAMED("tutorial", "Reference frame: %s", arm.getPlanningFrame().c_str());
  ROS_INFO_NAMED("tutorial", "End effector link: %s", arm.getEndEffectorLink().c_str());
  arm.setPlanningTime(0.050);
  arm.setPlannerId("RRTConnect");
  arm.setGoalTolerance(0.01);

  // pose1
  geometry_msgs::PoseStamped pose1;
  pose1.header.frame_id = "base_link";
  pose1.pose.position.x = 0.20;
  pose1.pose.position.y = 0.00;
  pose1.pose.position.z = 0.35;
  pose1.pose.orientation.w = 1.0;

  // pose2
  geometry_msgs::PoseStamped pose2;
  pose2.header.frame_id = "base_link";
  pose2.pose.position.x = 0.30;
  pose2.pose.position.y = 0.10;
  pose2.pose.position.z = 0.35;
  pose2.pose.orientation.w = 1.0;

  // pose3
  geometry_msgs::PoseStamped pose3;
  pose3.header.frame_id = "base_link";
  pose3.pose.position.x = 0.40;
  pose3.pose.position.y = 0.00;
  pose3.pose.position.z = 0.35;
  pose3.pose.orientation.w = 1.0;

  // pose4
  geometry_msgs::PoseStamped pose4;
  pose4.header.frame_id = "base_link";
  pose4.pose.position.x = 0.30;
  pose4.pose.position.y = -0.10;
  pose4.pose.position.z = 0.35;
  pose4.pose.orientation.w = 1.0;

  moveit::planning_interface::MoveItErrorCode ret;

  ROS_INFO("move to WP1");
  arm.setPoseTarget(pose1);
  ret = arm.move();
  if (!ret) {
    ROS_WARN("Fail: %i", ret.val);
  }
  ros::Duration(0.5).sleep();

  ROS_INFO("move to WP2");
  arm.setPoseTarget(pose2);
  ret = arm.move();
  if (!ret) {
    ROS_WARN("Fail: %i", ret.val);
  }
  ros::Duration(0.5).sleep();

  ROS_INFO("move to WP3");
  arm.setPoseTarget(pose3);
  ret = arm.move();
  if (!ret) {
    ROS_WARN("Fail: %i", ret.val);
  }
  ros::Duration(0.5).sleep();

  ROS_INFO("move to WP4");
  arm.setPoseTarget(pose4);
  ret = arm.move();
  if (!ret) {
    ROS_WARN("Fail: %i", ret.val);
  }
  ros::Duration(0.5).sleep();

  ros::shutdown();
  return 0;
}