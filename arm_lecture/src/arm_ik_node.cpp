#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include "arm_ik.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "arm_ik_node");
  ros::NodeHandle nh;
  ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
  ros::Publisher point_pub = nh.advertise<geometry_msgs::PointStamped>("target_point", 1);

  ArmMock arm_mock(0.05, 0.05, 0.15, 0.15, 0.1);
  ArmSolver arm_solver(0.05, 0.05, 0.15, 0.15, 0.1);
  ArmSmooth arm_smooth;

  Angle4D init_angles;
  init_angles.angle1 = 0.0;
  init_angles.angle2 = 0.0;
  init_angles.angle3 = 0.0;
  init_angles.angle4 = 0.0;
  arm_smooth.setCurrentAngles(init_angles);

  ros::Rate loop_rate(10);
  while(ros::ok()){
    geometry_msgs::PointStamped target_point;
    target_point.header.stamp = ros::Time::now();
    target_point.header.frame_id = "base_link";
    target_point.point.x = 0.3;
    target_point.point.y = 0.1;
    target_point.point.z = 0.2;
    point_pub.publish(target_point);
    Angle4D angles;
    if(!arm_solver.solve(target_point.point, 1.5707, angles)){
      ROS_INFO("can not solve");
      break;
    }
    arm_smooth.setTargetAngles(angles);
    for(int i = 0; i<20; i++){
      arm_mock.setAngle(arm_smooth.output((float)i/10));
      joint_state_pub.publish(arm_mock.getJointState());
      ros::spinOnce();
      loop_rate.sleep();
    }

    target_point.point.x = 0.2;
    target_point.point.y = -0.1;
    target_point.point.z = 0.3;
    point_pub.publish(target_point);
    if(!arm_solver.solve(target_point.point, 1.5707, angles)){
      ROS_INFO("can not solve");
      break;
    }
    arm_smooth.setTargetAngles(angles);
    for(int i = 0; i<20; i++){
      arm_mock.setAngle(arm_smooth.output((float)i/10));
      joint_state_pub.publish(arm_mock.getJointState());
      ros::spinOnce();
      loop_rate.sleep();
    }
  }
  return 0;
}