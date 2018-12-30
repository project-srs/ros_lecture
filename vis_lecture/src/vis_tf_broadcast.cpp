#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>

#include <string>
#include <math.h>

int main(int argc, char **argv){
  ros::init(argc, argv, "vis_tf_publisher");
  ros::NodeHandle nh;

  ros::Rate loop_rate(10); 
  int count=0;
  while (ros::ok()){
    float xyz[3]={0,0,0};
    xyz[0]=cos((float)count/5.0);
    xyz[1]=sin((float)count/5.0);

    std::string source_frame="base_link";
    std::string target_frame="body_link";
    geometry_msgs::Pose t_pose;
    t_pose.position.x=xyz[0];
    t_pose.position.y=xyz[1];
    t_pose.orientation.w=1.0;

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    poseMsgToTF(t_pose, transform);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), source_frame, target_frame));

    count++;
    ros::spinOnce();
    loop_rate.sleep();
  } 
  return 0;
}

