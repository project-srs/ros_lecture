#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <vision_msgs/Detection2DArray.h>

class ColorDetector{

public:
  ColorDetector()
    : nh_(""), pnh_("~"), it_(nh_)
  {
    camera_sub_ = it_.subscribeCamera("/head_camera/image_raw", 1, &ColorDetector::imageCallback, this);
    detect_pub_ = nh_.advertise<vision_msgs::Detection2DArray>("detection", 1);
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& image_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
    // get paramater
    std::string debug_output = "";
    pnh_.getParamCached("debug_output", debug_output);

    int h_min = 0;
    pnh_.getParamCached("h_min", h_min);
    int h_max = 180;
    pnh_.getParamCached("h_max", h_max);
    int s_min = 0;
    pnh_.getParamCached("s_min", s_min);
    int s_max = 255;
    pnh_.getParamCached("s_max", s_max);
    int v_min = 0;
    pnh_.getParamCached("v_min", v_min);
    int v_max = 255;
    pnh_.getParamCached("v_max", v_max);

    cv::Mat original_image;
    cv_bridge::CvImagePtr input_bridge;
    try {
      input_bridge = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
      original_image = input_bridge->image;
    }
    catch (cv_bridge::Exception& ex){
      ROS_ERROR("[draw_frames] Failed to convert image");
      return;
    }
    if (debug_output == "input") {
      cv::imshow("Debug", original_image);  
      cv::waitKey(1);  
    }

    // bluer
    cv::Mat blur_image;
    cv::Size ksize = cv::Size(3, 3);
    cv::GaussianBlur(original_image, blur_image, ksize, 0);
    if (debug_output == "blue") {
      cv::imshow("Debug", blur_image);
      cv::waitKey(1);  
    }

    // convert RBG -> HSV
    cv::Mat hsv_image;
    cv::cvtColor(blur_image, hsv_image, CV_BGR2HSV);

    // hsv range filter
    cv::Mat hsv_range_image;
    cv::Scalar min_range = cv::Scalar(h_min, s_min, v_min);
    cv::Scalar max_range = cv::Scalar(h_max, s_max, v_max);
    cv::inRange(hsv_image, min_range, max_range, hsv_range_image);
    if (debug_output == "range") {
      cv::Mat masked_image;
      original_image.copyTo(masked_image, hsv_range_image);
      cv::imshow("Debug", masked_image);
      cv::waitKey(1);  
    }

    // opening
    cv::Mat mid_mask;
    cv::Mat opened_mask;
    cv::dilate(hsv_range_image, mid_mask, cv::Mat(), cv::Point(-1,-1), 2);
    cv::erode(mid_mask, opened_mask, cv::Mat(), cv::Point(-1,-1), 2);
    if (debug_output == "opening") {
      cv::Mat masked_image;
      original_image.copyTo(masked_image, opened_mask);
      cv::imshow("Debug", masked_image);
      cv::waitKey(1);  
    }

    // get contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(opened_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    if (debug_output == "contour") {
      cv::Mat overlay_image;
      original_image.copyTo(overlay_image);
      for(auto c: contours){
        cv::polylines(overlay_image, c, true, cv::Scalar(255, 0, 0), 2, cv::LINE_AA);
      }
      cv::imshow("Debug", overlay_image);
      cv::waitKey(1);  
    }

    // get convex
    std::vector<std::vector<cv::Point>> approxes;
    for(auto c: contours){
      float area = cv::contourArea(c);
      if(area < 100){
        continue;
      }
      std::vector<cv::Point> approx;
      cv::convexHull(c, approx);
      approxes.push_back(approx);
    }
    if (debug_output == "convex") {
      cv::Mat overlay_image;
      original_image.copyTo(overlay_image);
      for(auto a: approxes){
        cv::polylines(overlay_image, a, true, cv::Scalar(255, 0, 0), 2, cv::LINE_AA);
      }
      cv::imshow("Debug", overlay_image);
      cv::waitKey(1);  
    }

    // get boounding box
    std::vector<cv::Rect> bboxes;
    for(auto a: approxes){
      cv::Rect rect = cv::boundingRect(a);
      bboxes.push_back(rect);
    }
    if (debug_output == "bbox") {
      cv::Mat overlay_image;
      original_image.copyTo(overlay_image);
      for(auto r: bboxes){
        cv::rectangle(overlay_image, r.tl(), r.br(), cv::Scalar(255, 0, 0), 2, cv::LINE_AA);
      }
      cv::imshow("Debug", overlay_image);
      cv::waitKey(1);  
    }

    // rotation rect
    std::vector<cv::RotatedRect> rects;
    for(auto a: approxes){
      cv::RotatedRect rect = cv::minAreaRect(a);
      rects.push_back(rect);
    }
    if (debug_output == "rect") {
      cv::Mat overlay_image;
      original_image.copyTo(overlay_image);
      for(auto r: rects){
        std::vector<cv::Point2f> points(4);
        r.points(points.data());
        std::vector<cv::Point> draw_points;
        for(auto p : points){
          draw_points.push_back(p);
        }
        cv::polylines(overlay_image, draw_points, true, cv::Scalar(255, 0, 0), 2, cv::LINE_AA);
      }
      cv::imshow("Debug", overlay_image);
      cv::waitKey(1);  
    }

    // result
    vision_msgs::Detection2DArray output;
    output.header = image_msg->header;
    for(auto r: bboxes){
      vision_msgs::Detection2D detect;
      detect.bbox.center.x = r.x + r.width / 2;
      detect.bbox.center.y = r.y + r.height / 2;
      detect.bbox.size_x = r.width;
      detect.bbox.size_y = r.height;
      output.detections.push_back(detect);
    }
    detect_pub_.publish(output);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher detect_pub_;
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber camera_sub_;
};

int main(int argc, char** argv){
  ros::init(argc, argv, "cam_color_detector");
  ColorDetector detector;
  ros::spin();
}