#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <gazebo_msgs/ModelStates.h>
#include <vision_msgs/Detection3DArray.h>

class ModelReader{

public:
  ModelReader()
    : nh_(""), pnh_("~")
  {
    detect_pub_ = nh_.advertise<vision_msgs::Detection3DArray>("objects", 1);
    model_sub_ = nh_.subscribe("/gazebo/model_states", 1, &ModelReader::modelsCallback, this);
    timer_ = nh_.createTimer(ros::Duration(0.1), &ModelReader::timerCallback, this);
    pnh_.getParamCached("target_list", target_list_);
    std::string str = "";
    for(auto t: target_list_){
      str += t+", ";
    }
    ROS_INFO("target %s", str.c_str());
  }

  void modelsCallback(const gazebo_msgs::ModelStates& model_msg){    
    last_model_=model_msg;
  }

  void timerCallback(const ros::TimerEvent& e){
    vision_msgs::Detection3DArray output;
    output.header.stamp = e.current_real;

    for(auto t : target_list_){
      auto itr = find(last_model_.name.begin(), last_model_.name.end(), t);
      if(itr == last_model_.name.end()){
        ROS_INFO_THROTTLE(2.0, "not match %s\n", t.c_str());
      }
      else{
        vision_msgs::Detection3D detect;
        detect.header.frame_id = "world";
        detect.header.stamp = e.current_real;
        int index = std::distance(last_model_.name.begin(), itr);
        detect.bbox.center = last_model_.pose[index];
        output.detections.push_back(detect);
      }
    }
    detect_pub_.publish(output);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher detect_pub_;
  ros::Subscriber model_sub_;
  gazebo_msgs::ModelStates last_model_;
  ros::Timer timer_;
  std::vector<std::string> target_list_;
};

int main(int argc, char** argv){
  ros::init(argc, argv, "cam_model_object");
  ModelReader model_reader;
  ros::spin();
}
