#include "point_tool.h"

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreEntity.h>
#include <ros/ros.h>
#include <rviz/viewport_mouse_event.h>
#include <rviz/visualization_manager.h>
#include <rviz/geometry.h>
#include <rviz/ogre_helpers/shape.h>
#include <rviz/frame_manager.h>
#include <geometry_msgs/PointStamped.h>
#include <pluginlib/class_list_macros.h>

namespace plugin_lecture{

PointTool::PointTool() : nh_() {
  shortcut_key_ = 'm';
}

PointTool::~PointTool(){
}

void PointTool::onInitialize(){
  vis_shape_.reset(new rviz::Shape(rviz::Shape::Cylinder, scene_manager_));
  Ogre::Vector3 shape_pos(0,2,0);
  vis_shape_->setPosition(shape_pos); 
  Ogre::Quaternion  shape_q(0.7, 0.7, 0, 0);
  vis_shape_->setOrientation(shape_q);   
  vis_shape_->setColor(0, 0, 1, 1);
  vis_shape_->getRootNode()->setVisible(false);

  point_pub_ = nh_.advertise<geometry_msgs::PointStamped>( "point", 10 ); 
}

void PointTool::activate(){
  vis_shape_->setColor(0, 0, 1, 1);
  vis_shape_->getRootNode()->setVisible(true);
}

void PointTool::deactivate(){
  vis_shape_->setColor(0.5, 0.5, 0.5, 1);
}

int PointTool::processMouseEvent( rviz::ViewportMouseEvent& event ){
  Ogre::Vector3 intersection;
  Ogre::Plane ground_plane( Ogre::Vector3::UNIT_Z, 0.0f );
  if( rviz::getPointOnPlaneFromWindowXY(event.viewport, ground_plane, event.x, event.y, intersection)){
    vis_shape_->setPosition( intersection );
    if(event.leftDown()){
      geometry_msgs::PointStamped point_msg;
      point_msg.header.frame_id = context_->getFrameManager()->getFixedFrame();
      point_msg.header.stamp = ros::Time::now();
      point_msg.point.x = intersection.x;
      point_msg.point.y = intersection.y;
      point_msg.point.z = intersection.z;
      point_pub_.publish(point_msg);
      return Render|Finished;
    }
  }
  return Render;
}

} // namespace plugin_lecture
PLUGINLIB_EXPORT_CLASS(plugin_lecture::PointTool, rviz::Tool )
