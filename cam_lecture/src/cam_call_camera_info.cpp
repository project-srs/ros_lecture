#include <ros/ros.h>
#include <sensor_msgs/SetCameraInfo.h>

bool get_camera_info(sensor_msgs::CameraInfo& info, ros::NodeHandle pn){
  //basic config
  std::string camera_name;
  int image_width, image_height;
  //camera config
  int camera_matrix_rows, camera_matrix_cols;
  std::vector<double>camera_matrix_data;
  //distortion config
  std::string distortion_model;
  int distortion_rows, distortion_cols;
  std::vector<double>distortion_data;
  //rectification config
  int rectification_rows, rectification_cols;
  std::vector<double>rectification_data;
  //projection config
  int projection_rows, projection_cols; 
  std::vector<double>projection_data;
  
  if(!pn.getParam("camera_name", camera_name))return false;
  if(!pn.getParam("image_width", image_width))return false;
  if(!pn.getParam("image_height", image_height))return false;

  if(!pn.getParam("camera_matrix/rows", camera_matrix_rows))return false;
  if(!pn.getParam("camera_matrix/cols", camera_matrix_cols))return false;
  if(!pn.getParam("camera_matrix/data", camera_matrix_data))return false;

  if(!pn.getParam("distortion_model", distortion_model))return false;
  if(!pn.getParam("distortion_coefficients/rows", distortion_rows))return false;
  if(!pn.getParam("distortion_coefficients/cols", distortion_cols))return false;
  if(!pn.getParam("distortion_coefficients/data", distortion_data))return false;
  
  if(!pn.getParam("rectification_matrix/rows", rectification_rows))return false;
  if(!pn.getParam("rectification_matrix/cols", rectification_cols))return false;
  if(!pn.getParam("rectification_matrix/data", rectification_data))return false;

  if(!pn.getParam("projection_matrix/rows", projection_rows))return false;
  if(!pn.getParam("projection_matrix/cols", projection_cols))return false;
  if(!pn.getParam("projection_matrix/data", projection_data))return false;

  info.header.frame_id=camera_name;
  info.width=image_width;
  info.height=image_height;

  if(camera_matrix_rows==3 && camera_matrix_cols==3 && camera_matrix_data.size()==9){
    for(int i=0;i<9;i++)info.K[i]=camera_matrix_data[i];
  }
  else return false;

  if(distortion_rows==1 && distortion_cols==5 && distortion_data.size()==5){
    info.D=distortion_data;
  }
  else return false;
  info.distortion_model=distortion_model;

  if(rectification_rows==3 && rectification_cols==3 && rectification_data.size()==9){
    for(int i=0;i<9;i++)info.R[i]=rectification_data[i];
  }
  else return false;

  if(projection_rows==3 && projection_cols==4 && projection_data.size()==12){
    for(int i=0;i<12;i++)info.P[i]=projection_data[i];
  }
  else return false;

  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cam_call_camera_info");
  ros::NodeHandle n;
  ros::NodeHandle pn("~"); 

  ros::Duration(5.0).sleep();
  sensor_msgs::CameraInfo info;
  if(!get_camera_info(info, pn)){
    ROS_ERROR("Can not find rosparam");
    return -1;
  }

  ros::ServiceClient client = n.serviceClient<sensor_msgs::SetCameraInfo>("set_camera_info");
  sensor_msgs::SetCameraInfo srv;
  srv.request.camera_info=info;
  if(!client.call(srv)){
    ROS_ERROR("Fail: service call");
    return -1;
  }
  if(!srv.response.success){
    ROS_ERROR("Fail: responce is bad");
    return -1;
  }
  ROS_INFO("Success: set_camera_info");    
  return 0;
}