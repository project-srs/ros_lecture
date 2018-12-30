#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
  
#include <string>
#include <math.h>

int main(int argc, char **argv){
  ros::init(argc, argv, "vis_tf_listen");
  ros::NodeHandle nh;

  tf::TransformListener ln;

  ros::Rate loop_rate(10); 
  while (ros::ok()){
    geometry_msgs::PoseStamped source_pose;
    source_pose.header.frame_id="body_link";
    source_pose.header.stamp=ros::Time(0);
    source_pose.pose.orientation.w=1.0;

    geometry_msgs::PoseStamped target_pose;
    std::string target_frame="base_link";
    try{
      ln.waitForTransform(source_pose.header.frame_id, target_frame, ros::Time(0), ros::Duration(1.0));
      ln.transformPose(target_frame, source_pose, target_pose);

      ROS_INFO("x:%+5.2f, y:%+5.2f,z:%+5.2f",target_pose.pose.position.x,target_pose.pose.position.y,target_pose.pose.position.z);
    }
    catch(...){
      ROS_INFO("tf error");
    }
    ros::spinOnce();
    loop_rate.sleep();
  } 
  return 0;
}
