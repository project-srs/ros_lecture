#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/image_encodings.h>
#include <jsk_recognition_msgs/ObjectArray.h>

class FrameDrawer{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber sub_;
  ros::Subscriber sub2_;
  image_transport::Publisher pub_;
  tf::TransformListener tf_listener_;
  image_geometry::PinholeCameraModel cam_model_;
  std::vector<std::string> frame_ids_;
  jsk_recognition_msgs::ObjectArray last_objects_;

public:
  FrameDrawer(const std::vector<std::string>& frame_ids)
    : it_(nh_), frame_ids_(frame_ids)
  {
    std::string image_topic = nh_.resolveName("image");
    sub_ = it_.subscribeCamera(image_topic, 1, &FrameDrawer::imageCb, this);
    pub_ = it_.advertise("output_image", 1);
    sub2_ = nh_.subscribe("objects", 1, &FrameDrawer::objectCb, this);
  }

  void objectCb(const jsk_recognition_msgs::ObjectArray object_msg){
    last_objects_=object_msg;
  }

  void imageCb(const sensor_msgs::ImageConstPtr& image_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
    static tf::TransformListener tflistener;

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

    cam_model_.fromCameraInfo(info_msg);
    for(int i=0;i<last_objects_.objects.size();i++){
      geometry_msgs::PoseStamped source_pose;
      source_pose.header.frame_id=last_objects_.header.frame_id;
      source_pose.pose.position.x=last_objects_.objects[i].dimensions.x;
      source_pose.pose.position.y=last_objects_.objects[i].dimensions.y;
      source_pose.pose.position.z=last_objects_.objects[i].dimensions.z;
      source_pose.pose.orientation.w=1.0;
      geometry_msgs::PoseStamped target_pose;
      try{
        tflistener.waitForTransform(cam_model_.tfFrame(), source_pose.header.frame_id, ros::Time(0), ros::Duration(1.0));
        tflistener.transformPose(cam_model_.tfFrame(),ros::Time(0),source_pose,source_pose.header.frame_id,target_pose);
      }
      catch(...){
        ROS_INFO("tf error");
      }

      if(0.1<target_pose.pose.position.z && target_pose.pose.position.z<10){
        cv::Point3d pt_cv1(target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z);
        cv::Point2d uv1 = cam_model_.project3dToPixel(pt_cv1);
        cv::circle(image, cv::Point(uv1.x,uv1.y), 30, CV_RGB(255,0,0), 5);
        cv:putText(image, last_objects_.objects[i].name.c_str(), cv::Point(uv1.x-50, uv1.y-40), cv::FONT_HERSHEY_SIMPLEX, 1.5, CV_RGB(255,0,0), 3);

        //printf("name: %s\n", last_objects_.objects[i].name.c_str());
        //printf("origin: x:%f, y:%f, z:%f\n", last_objects_.objects[i].dimensions.x, last_objects_.objects[i].dimensions.y, last_objects_.objects[i].dimensions.z);
        //printf("trans : x:%f, y:%f, z:%f\n", target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z);
        //printf("image : x:%f, y:%f\n", uv1.x, uv1.y);
      }
    }
    pub_.publish(input_bridge->toImageMsg());
  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "cam_display_objects");
  std::vector<std::string> frame_ids(argv + 1, argv + argc);
  FrameDrawer drawer(frame_ids);
  ros::spin();
}