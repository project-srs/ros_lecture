#include <bits/stdc++.h>
#include <cmath>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <jsk_recognition_msgs/PolygonArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseArray.h>

#include <fstream>
#include <string>
#include<algorithm>

struct CurveFeature{
  geometry_msgs::Point32 point;
  float angle{0.0f};
  float sin_value{0.0f};
  int reference_index{0};
};

struct SearchItem{
  float x;
  float y;
  float a;
  void becomeHalf(void){
    x *=0.5;
    y *=0.5;
    a *=0.5;
  }
};
struct DetectedObject{
  geometry_msgs::Point32 point;
  float angle{0.0f};
  int reference_index{0};
  DetectedObject add(SearchItem shift){
    DetectedObject output;
    output.point.x = point.x + shift.x;
    output.point.y = point.y + shift.y;
    output.point.z = point.z;
    output.angle = angle + shift.a;
    output.reference_index = reference_index;
    return output;
  }
};

struct Point2D{
  float x{0.0f};
  float y{0.0f};
};
struct TempShape{
  std::vector<Point2D> points;
  std::vector<Point2D> convert(DetectedObject obj){
    std::vector<Point2D> output;
    for(auto p : points){
      Point2D cp;
      cp.x = cos(obj.angle) * p.x - sin(obj.angle) * p.y + obj.point.x;
      cp.y = sin(obj.angle) * p.x + cos(obj.angle) * p.y + obj.point.y;
      output.push_back(cp);
    }
    return output;
  }
};

class LaserGroup{
public:
  LaserGroup() : nh_(), pnh_("~") {
    laser_sub_ = nh_.subscribe("/laser/scan", 10, &LaserGroup::laserCallback, this);
    markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("marker_array", 1);
    pose_array_pub_ = nh_.advertise<geometry_msgs::PoseArray>("detected_poses", 1);
  }

  void laserCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in){
    // convert scan to pointcloud
    sensor_msgs::PointCloud cloud;
    projector_.projectLaser(*scan_in, cloud);

    // extract curve feature
    std::vector<CurveFeature> curve_feature = extractFeatureFromPc(cloud, 0.4, 0.7);

    // convert to object
    std::vector<DetectedObject> detected_objects = extractObjectFromFeatures(curve_feature);

    // match to pc
    TempShape original_temp = getOriginalTemplate();
    std::vector<DetectedObject> matched_objects = matchObject(original_temp, cloud, detected_objects, 0.02);

    // output
    pose_array_pub_.publish(generatePoses(matched_objects, scan_in->header));

    // visualize
    visualization_msgs::MarkerArray markers;
    for(auto m :generateMarkers(curve_feature)){
      markers.markers.push_back(m);
    }
    for(auto m: generateMarkers(original_temp, detected_objects, "detected_objects")){
      markers.markers.push_back(m);
    }
    for(auto m: generateMarkers(original_temp, matched_objects, "matched_objects")){
      markers.markers.push_back(m);
    }
    markers_pub_.publish(markers);
  }

  std::vector<CurveFeature> extractFeatureFromPc(sensor_msgs::PointCloud cloud, float jump_threshold, float sin_threshold){
    std::vector<CurveFeature> output;
    if(cloud.points.size() < 7 ){
      return output;
    }

    for(size_t i = 3; i+3< cloud.points.size(); i++){
      // jump detect
      auto getDis = [](geometry_msgs::Point32 a, geometry_msgs::Point32 b){
        float dx1 = b.x - a.x;
        float dy1 = b.y - a.y;
        return sqrt(dx1*dx1+dy1*dy1);
      };
      const float dis1 = getDis(cloud.points[i-1], cloud.points[i+0]);
      const float dis2 = getDis(cloud.points[i+0], cloud.points[i+1]);
      if(jump_threshold < dis1 || jump_threshold < dis2){
        continue;
      }

      // get angle
      float angle_x = (cloud.points[i+2].x + cloud.points[i-2].x)/2 - cloud.points[i+0].x;
      float angle_y = (cloud.points[i+2].y + cloud.points[i-2].y)/2 - cloud.points[i+0].y;
      float angle = atan2(angle_y, angle_x);

      // get feature value
      auto getSin = [](geometry_msgs::Point32 a, geometry_msgs::Point32 b, geometry_msgs::Point32 c){
        float dx1 = b.x - a.x;
        float dy1 = b.y - a.y;
        float l1 = sqrt(dx1*dx1+dy1*dy1);
        float dx2 = c.x - b.x;
        float dy2 = c.y - b.y;
        float l2 = sqrt(dx2*dx2+dy2*dy2);
        return (dx1*dy2 - dy1*dx2)/(l1*l2);
      };
      float sinth1 = getSin(cloud.points[i-1], cloud.points[i+0], cloud.points[i+1]);
      float sinth2 = getSin(cloud.points[i-2], cloud.points[i+0], cloud.points[i+2]);
      float sinth3 = getSin(cloud.points[i-3], cloud.points[i+0], cloud.points[i+3]);

      const float sinth_min = std::min(std::min(sinth1, sinth2), sinth3);
      const float sinth_max = std::max(std::max(sinth1, sinth2), sinth3);

      float sin_value = 0.0f;
      if(sinth_min < - sin_threshold && sinth_max < -sin_threshold) sin_value = sinth_max;
      if(sin_threshold < sinth_min && sin_threshold < sinth_max) sin_value = sinth_min;
    
      if(sin_value != 0){
        CurveFeature feature;
        feature.point = cloud.points[i];
        feature.angle = angle;
        feature.reference_index = i;
        feature.sin_value = sin_value;
        output.push_back(feature);
      }
    }
    return output;
  }

  std::vector<DetectedObject> extractObjectFromFeatures(std::vector<CurveFeature> features){
    std::vector<DetectedObject> output;
    for(auto f : features){
      if(0 < f.sin_value){
        DetectedObject obj;
        obj.point = f.point;
        obj.angle = f.angle;
        obj.reference_index = f.reference_index;
        output.push_back(obj);
      }
    }
    return output;
  }

  std::vector<DetectedObject> matchObject(TempShape temp, sensor_msgs::PointCloud cloud, std::vector<DetectedObject> input, float threshold){
    std::vector<DetectedObject> output;
    for(auto in : input){
      sensor_msgs::PointCloud pc = extractPointCloud(cloud, in.reference_index, 10);
      DetectedObject current_object = in;
      std::vector<SearchItem> search_list = {
        {0, 0, 0},
        {0.02, 0, 0},
        {-0.02, 0, 0},
        {0, 0.02, 0},
        {0, -0.02, 0},
        {0, 0, 0.1},
        {0, 0, -0.1},
      }; 

      float last_score=0.0f;
      for(int i=0;i<6;i++){
        std::vector<float> scores;
        for(auto s: search_list){
          DetectedObject candidate = current_object.add(s);
          auto t = temp.convert(candidate);
          float score = getDifference(t, pc);
          scores.push_back(score);
        }
        std::vector<float>::iterator iter = std::min_element(scores.begin(), scores.end());
        size_t index = std::distance(scores.begin(), iter);
        // printf("score: [%lu]%f<-%f\n", index, scores[index], scores[0]);
        last_score = scores[index];
        current_object = current_object.add(search_list[index]);

        if(index == 0){
          for(auto& item : search_list){
            item.becomeHalf();
          }
        }
      }
      if(last_score < threshold){
        output.push_back(current_object);
      }
    }
    return output;
  }

  float getDifference(std::vector<Point2D> temp, sensor_msgs::PointCloud pc){    
    std::vector<float> scores;
    for(size_t i = 0; i < temp.size(); i++){
      float min_diff = 1000.0f;
      for(size_t j = 0; j < pc.points.size(); j++){
        float dx = temp[i].x - pc.points[j].x;
        float dy = temp[i].y - pc.points[j].y;
        float diff = sqrt(dx*dx+dy*dy);
        min_diff = std::min(min_diff, diff);
      }
      scores.push_back(min_diff);
    }
    float sum = 0.0f;
    for(auto s : scores){
      sum += s;
    }
    return sum / scores.size();
  }

  sensor_msgs::PointCloud extractPointCloud(sensor_msgs::PointCloud pc, int target_index, int side_width){
    sensor_msgs::PointCloud output;
    output.header = pc.header;
    for(size_t i=0; i<pc.points.size();i++){
      if(i < target_index - side_width || target_index + side_width < i){
        continue;
      }
      output.points.push_back(pc.points[i]);
    }
    return output;
  }

  TempShape getOriginalTemplate(void){
    TempShape output;
    std::vector<float> x_list = {0.150, 0.150, 0.075, 0, 0.075, 0.150, 0.150};
    std::vector<float> y_list = {0.250, 0.150, 0.075, 0, -0.075, -0.150, -0.250};
    size_t list_size = std::min(x_list.size(), y_list.size());

    for (size_t i=0;i<list_size;i++){
      Point2D p;
      p.x = x_list[i];
      p.y = y_list[i];
      output.points.push_back(p);
    }
    return output;
  }

  visualization_msgs::Marker generateDeleteMarker(int index){
    visualization_msgs::Marker marker;
    // marker0
    marker.header.frame_id = "laser_link";
    marker.header.stamp = ros::Time::now();
    marker.ns = "matched";
    marker.lifetime = ros::Duration();
    marker.id = index;

    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::DELETE;
    marker.scale.x = 0.02;
    marker.pose.orientation.w = 1.0;
    return marker;
  }

  geometry_msgs::PoseArray generatePoses(std::vector<DetectedObject> objects, std_msgs::Header header){
    geometry_msgs::PoseArray output;
    output.header = header;
    for(auto o : objects){
      geometry_msgs::Pose pose;
      pose.position.x = o.point.x;
      pose.position.y = o.point.y;
      pose.position.z = o.point.z;
      pose.orientation = tf::createQuaternionMsgFromYaw(o.angle);
      output.poses.push_back(pose);
    }
    return output;
  }

  std::vector<visualization_msgs::Marker> generateMarkers(std::vector<CurveFeature> features){
    std::vector<visualization_msgs::Marker> output;
    size_t index = 0;
    for(auto f: features){
      visualization_msgs::Marker marker;
      // marker0
      marker.header.frame_id = "laser_link";
      marker.header.stamp = ros::Time::now();
      marker.ns = "curve_feature";
      marker.id = index++;
      marker.lifetime = ros::Duration(0.2);

      marker.type = visualization_msgs::Marker::SPHERE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.scale.x = 0.15;
      marker.scale.y = 0.15;
      marker.scale.z = 0.15;

      marker.pose.position.x = f.point.x;
      marker.pose.position.y = f.point.y;
      marker.pose.position.z = f.point.z;
      marker.pose.orientation.w = 1.0;

      if(0 < f.sin_value){
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;
      } else {
        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
        marker.color.a = 1.0f;
      }

      output.push_back(marker);
    }
    return output;
  }

  std::vector<visualization_msgs::Marker> generateMarkers(TempShape temp, std::vector<DetectedObject> objects, std::string ns){
    std::vector<visualization_msgs::Marker> output;
    size_t index = 0;
    for(auto obj: objects){
      visualization_msgs::Marker marker;
      // marker0
      marker.header.frame_id = "laser_link";
      marker.header.stamp = ros::Time::now();
      marker.ns = ns;
      marker.id = index++;
      marker.lifetime = ros::Duration(0.2);

      marker.type = visualization_msgs::Marker::LINE_STRIP;
      marker.action = visualization_msgs::Marker::ADD;
      marker.scale.x = 0.02;
      marker.pose.position.x = obj.point.x;
      marker.pose.position.y = obj.point.y;
      marker.pose.position.z = obj.point.z;
      marker.pose.orientation = tf::createQuaternionMsgFromYaw(obj.angle);

      marker.points.clear();
      for(auto t: temp.points){
        geometry_msgs::Point p;
        p.x = t.x;
        p.y = t.y;
        p.z = 0.0;
        marker.points.push_back(p);
      }
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0f;
      output.push_back(marker);
    }
    return output;
  }
  
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  laser_geometry::LaserProjection projector_;
  // tf::TransformListener listener_;
  ros::Subscriber laser_sub_;
  ros::Publisher markers_pub_;
  ros::Publisher pose_array_pub_;
};


int main(int argc, char** argv){
  ros::init(argc, argv, "laser_detector");
  LaserGroup laser_group;
  ros::spin();
  return 0;
}
