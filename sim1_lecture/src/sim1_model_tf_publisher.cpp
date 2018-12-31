#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <tf/transform_broadcaster.h>

#include <string>
#include <math.h>

gazebo_msgs::ModelStates last_model_msg;
void models_callback(const gazebo_msgs::ModelStates& model_msg){
  last_model_msg=model_msg;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "sim1_model_tf_publisher");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string model_name="";
  std::string world_frame="";
  std::string base_frame="";
  float hz=20;

  //rosparam
  pnh.getParam("model_name",  model_name);
  pnh.getParam("world_frame", world_frame);
  pnh.getParam("base_frame",  base_frame);
  pnh.getParam("hz", hz);

  //subscriibe
  ros::Subscriber joy_sub   = nh.subscribe("/gazebo/model_states", 10, models_callback);

  tf::TransformBroadcaster br;
  ros::Rate loop_rate(hz); 
  while (ros::ok()){
    int model_size=last_model_msg.name.size();
    for(int i=0;i<model_size;i++){
      if(last_model_msg.name[i]==model_name){
        tf::Transform transform;
        tf::poseMsgToTF(last_model_msg.pose[i],transform);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), world_frame, base_frame));
      }
    }
    ros::spinOnce();
    loop_rate.sleep();
  } 
  return 0;
}
