#include <ros/ros.h>
#include "arm_ik.h"

int main(int argc, char **argv) {
  ArmMock arm_mock(0.05, 0.05, 0.15, 0.15, 0.1);
  ArmSolver arm_solver(0.05, 0.05, 0.15, 0.15, 0.1);

  geometry_msgs::Point target_point;
  target_point.x = 0.3;
  target_point.y = 0.0;
  target_point.z = 0.2;
  ROS_INFO("input pos: %f, %f, %f", target_point.x, target_point.y, target_point.z);
    
  Angle4D angles;
  if(arm_solver.solve(target_point, 1.5707, angles)){
    arm_mock.setAngle(angles);
    ROS_INFO("angles: %f, %f, %f, %f", angles.angle1, angles.angle2, angles.angle3, angles.angle4);
    geometry_msgs::Point output_point = arm_mock.getTargetPoint();
    ROS_INFO("input pos: %f, %f, %f", output_point.x, output_point.y, output_point.z);
  }
  else{
    ROS_INFO("can not solve");
  }
   
  return 0;
}