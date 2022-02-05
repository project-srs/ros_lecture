#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/image_encodings.h>
#include <vision_msgs/Detection2DArray.h>
#include <sensor_msgs/CameraInfo.h>

class Reprojector{
  ros::NodeHandle nh_;
  ros::Subscriber detection_sub_;
  ros::Subscriber camera_info_sub_;
public:
  Reprojector()
  {
    detection_sub_ = nh_.subscribe("detection", 1, &Reprojector::detectionCallback, this);
    camera_info_sub_ = nh_.subscribe("/head_camera/camera_info", 1, &Reprojector::cameraInfoCallback, this);
  }

  void cameraInfoCallback(const sensor_msgs::CameraInfo msg){
    last_camera_info_ = msg;
  }

  void detectionCallback(const vision_msgs::Detection2DArray msg){
    image_geometry::PinholeCameraModel cam_model;
    cam_model.fromCameraInfo(last_camera_info_);

    for(auto d : msg.detections){
      cv::Point2d detect_uv;
      detect_uv.x = d.bbox.center.x;
      detect_uv.y = d.bbox.center.y;
      cv::Point3d ray = cam_model.projectPixelTo3dRay(detect_uv);
      printf("ray %f %f %f\n", ray.x, ray.y, ray.z);
    }
  }

  sensor_msgs::CameraInfo last_camera_info_;
};

int main(int argc, char** argv){
  ros::init(argc, argv, "cam_reprojection");
  Reprojector reprojector;
  ros::spin();
}