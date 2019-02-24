#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <jsk_recognition_msgs/ObjectArray.h>

#include <gazebo_msgs/ModelStates.h>

float publish_rate=20.0;
ros::Publisher poses_pub;
gazebo_msgs::ModelStates last_model;

void models_callback(const gazebo_msgs::ModelStates& model_msg){    
  last_model=model_msg;
}

void poses_callback(const ros::TimerEvent&){
  int model_size=last_model.name.size();
  jsk_recognition_msgs::ObjectArray poses_msg;
  poses_msg.objects.resize(model_size);
  poses_msg.header.frame_id="world";

  for(int i=0;i<model_size;i++){
    poses_msg.objects[i].name=last_model.name[i];
    poses_msg.objects[i].dimensions.x=last_model.pose[i].position.x;
    poses_msg.objects[i].dimensions.y=last_model.pose[i].position.y;
    poses_msg.objects[i].dimensions.z=last_model.pose[i].position.z;
  }
  poses_pub.publish(poses_msg);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "cam_model_objects");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  //rosparam
  pnh.getParam("publish_rate", publish_rate);

  //publisher
  poses_pub = nh.advertise<jsk_recognition_msgs::ObjectArray>("objects", 1);

  //subscriibe
  ros::Subscriber model_sub   = nh.subscribe("/gazebo/model_states", 1, models_callback);

  //timer
  ros::Timer poses_timer = nh.createTimer(ros::Duration(1.0/publish_rate), poses_callback);
  ros::spin();
  return 0;
}
