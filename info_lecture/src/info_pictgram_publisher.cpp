#include <jsk_rviz_plugins/Pictogram.h>
#include <ros/ros.h>

#include <string>

int main(int argc, char **argv) {
  ros::init(argc, argv, "info_pictgram_publisher");
  ros::NodeHandle nh;

  // publisher
  ros::Publisher pictgram_pub = nh.advertise<jsk_rviz_plugins::Pictogram>("pictgram", 1);

  ros::Rate loop_rate(1);
  int type_count = 0;
  while (ros::ok()) {
    jsk_rviz_plugins::Pictogram pictgram;
    pictgram.header.frame_id = "base_link";
    pictgram.header.stamp = ros::Time::now();
    pictgram.pose.position.z = 0.3;
    pictgram.pose.orientation.y = -0.71;
    pictgram.pose.orientation.w = 0.71;
    pictgram.action = jsk_rviz_plugins::Pictogram::ADD;
    pictgram.color.r = 1.0;
    pictgram.color.a = 1.0;

    if(type_count%4 == 0){
      pictgram.mode = jsk_rviz_plugins::Pictogram::PICTOGRAM_MODE;
      pictgram.character = "fa-angle-down";
      pictgram.size = 0.5;
    }
    else if(type_count%4 == 1){
      pictgram.mode = jsk_rviz_plugins::Pictogram::PICTOGRAM_MODE;
      pictgram.character = "tag";
      pictgram.size = 0.5;
    }
    else if(type_count%4 == 2){
      pictgram.mode = jsk_rviz_plugins::Pictogram::PICTOGRAM_MODE;
      pictgram.character = "down";
      pictgram.size = 0.5;
    }
    else{
      pictgram.mode = jsk_rviz_plugins::Pictogram::STRING_MODE;
      pictgram.character = "CHAR";
      pictgram.size = 0.2;
    }    
    pictgram_pub.publish(pictgram);
    type_count++;
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
