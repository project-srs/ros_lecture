#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/image_encodings.h>
#include <vision_msgs/Detection3DArray.h>

class FrameDrawer{
public:
  FrameDrawer() : it_(nh_)
  {
    camera_sub_ = it_.subscribeCamera("/head_camera/image_raw", 1, &FrameDrawer::imageCb, this);
    camera_pub_ = it_.advertise("output_image", 1);
    object_sub_ = nh_.subscribe("objects", 1, &FrameDrawer::objectCb, this);
  }

  void objectCb(const vision_msgs::Detection3DArray object_msg){
    last_objects_=object_msg;
  }

  void imageCb(const sensor_msgs::ImageConstPtr& image_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
    cv::Mat image;
    cv_bridge::CvImagePtr input_bridge;
    try {
      input_bridge = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
      image = input_bridge->image;
    }
    catch (cv_bridge::Exception& ex){
      ROS_ERROR("[draw_frames] Failed to convert image");
      return;
    }

    image_geometry::PinholeCameraModel cam_model;
    cam_model.fromCameraInfo(info_msg);
    for(int i=0;i<last_objects_.detections.size();i++){
      geometry_msgs::PoseStamped source_pose;
      source_pose.header = last_objects_.detections[i].header;
      source_pose.pose = last_objects_.detections[i].bbox.center;
      geometry_msgs::PoseStamped target_pose;
      try{
        tf_listener_.waitForTransform(cam_model.tfFrame(), source_pose.header.frame_id, ros::Time(0), ros::Duration(1.0));
        tf_listener_.transformPose(cam_model.tfFrame(),ros::Time(0),source_pose,source_pose.header.frame_id,target_pose);
      }
      catch(...){
        ROS_INFO("tf error %s->%s", cam_model.tfFrame().c_str(), source_pose.header.frame_id.c_str());
      }

      if(0.1<target_pose.pose.position.z && target_pose.pose.position.z<10){
        cv::Point3d pt_cv1(target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z);
        cv::Point2d uv1 = cam_model.project3dToPixel(pt_cv1);
        cv::Point2d uv2 = cam_model.unrectifyPoint(uv1); // position on rectify image -> on unrectify image
        cv::circle(image, cv::Point(uv2.x,uv2.y), 30, CV_RGB(255,0,0), 5);
      }
    }
    camera_pub_.publish(input_bridge->toImageMsg());
  }

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber camera_sub_;
  ros::Subscriber object_sub_;
  image_transport::Publisher camera_pub_;
  tf::TransformListener tf_listener_;
  vision_msgs::Detection3DArray last_objects_;
};

int main(int argc, char** argv){
  ros::init(argc, argv, "cam_display_objects");
  FrameDrawer drawer;
  ros::spin();
}