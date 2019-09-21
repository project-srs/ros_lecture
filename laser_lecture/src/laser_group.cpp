#include <bits/stdc++.h>
#include <cmath>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <jsk_recognition_msgs/PolygonArray.h>

class LaserGroup{
public:
  LaserGroup() : nh_(), pnh_("~") {
    polygon_pub_ = nh_.advertise<jsk_recognition_msgs::PolygonArray>("obstacle_area", 10);
    chatter_sub_ = nh_.subscribe("scan", 10, &LaserGroup::laserCallback, this);
    thresholds_.resize(4);
    thresholds_[0] = 0.15;
    thresholds_[1] = 0.45;
    thresholds_[2] = 0.55;
    thresholds_[3] = 1.0;
    pnh_.getParam("thresholds0", thresholds_[0]);
    pnh_.getParam("thresholds1", thresholds_[1]);
    pnh_.getParam("thresholds2", thresholds_[2]);
    pnh_.getParam("thresholds3", thresholds_[3]);
  }

  void laserCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in){
    if(!listener_.waitForTransform(
      scan_in->header.frame_id,
      "/base_link",
      scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
      ros::Duration(1.0))){
      return;
    }
    sensor_msgs::PointCloud cloud;
    projector_.transformLaserScanToPointCloud("/base_link",*scan_in, cloud, listener_);

    std::vector<std::vector<int> >counts;
    counts.resize(4);
    counts[0].resize(12);
    counts[1].resize(12);
    counts[2].resize(12);
    counts[3].resize(12);

    for(auto point : cloud.points){
      int length_index = getLengthIndex(point.x, point.y);
      int angle_index = getAngleIndex(point.x, point.y);
      if(0 <= length_index){
        counts[length_index][angle_index]++;
      }
    }
    jsk_recognition_msgs::PolygonArray p_msg;
    p_msg.header.frame_id = "base_link";
    p_msg.header.stamp = ros::Time::now();

    for(int i = 0; i < 4; i++){
      for(int j = 0; j < 12; j++){
        if(i == 0)p_msg.polygons.push_back(getPolygonFan(p_msg.header, j * M_PI / 6, M_PI / 6, 0.1, thresholds_[i]));
        else p_msg.polygons.push_back(getPolygonFan(p_msg.header, j * M_PI / 6, M_PI / 6, thresholds_[i - 1], thresholds_[i]));
        if(counts[i][j]>0)p_msg.likelihood.push_back(1.0);
        else p_msg.likelihood.push_back(0.0);
      }
    }
    polygon_pub_.publish(p_msg);
  }

  int getAngleIndex(float x, float y){
    float p_angle = atan2(y, x);
    int angle_index1 = p_angle / (M_PI / 12);
    int angle_index2;
    if(angle_index1 ==0)angle_index2 = 0;
    else if(angle_index1 > 0)angle_index2 = (angle_index1 + 1) / 2;
    else angle_index2 = 12 + ((angle_index1 - 1) / 2);
    return angle_index2;    
  }

  int getLengthIndex(float x, float y){
    float p_length = sqrt(x * x + y * y);
    for(int i = 0; i < 4; i++){
      if(p_length < thresholds_[i])return i;
    }
    return -1;
  }

  geometry_msgs::PolygonStamped getPolygonFan(std_msgs::Header header, float a_centor, float a_width, float l_min, float l_max){
    float diff = 0.005;
    geometry_msgs::PolygonStamped output;
    output.header = header;
    output.polygon.points.resize(11*2);
    for(int i = 0; i < 11; i++){
      float theta = a_centor + (a_width - 2 * diff / l_min) * (float)(i - 5)/ 10;
      output.polygon.points[i].x = (l_min + diff) * cos(theta);
      output.polygon.points[i].y = (l_min + diff) * sin(theta);
    }
    for(int i = 0; i < 11; i++){
      float theta = a_centor - (a_width - 2 * diff / l_max) * (float)(i - 5)/ 10;
      output.polygon.points[i + 11].x = (l_max - diff) * cos(theta);
      output.polygon.points[i + 11].y = (l_max - diff) * sin(theta);
    }
    return output;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  laser_geometry::LaserProjection projector_;
  tf::TransformListener listener_;
  ros::Subscriber chatter_sub_;
  ros::Publisher polygon_pub_;
  std::vector<float> thresholds_;
};

int main(int argc, char** argv){
  ros::init(argc, argv, "laser_group");
  LaserGroup laser_group;
  ros::spin();
  return 0;
}
