#include "point_display.h"

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <tf/transform_listener.h>
#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/frame_manager.h>
#include <pluginlib/class_list_macros.h>

namespace plugin_lecture{

PointDisplay::PointDisplay(){
  color_property_ = new rviz::ColorProperty( "Color", QColor( 200, 50, 50 ),
                                             "Color to draw the acceleration arrows.",
                                             this, SLOT( updateColorAndAlpha() ));

  alpha_property_ = new rviz::FloatProperty( "Alpha", 1.0,
                                             "0 is fully transparent, 1.0 is fully opaque.",
                                             this, SLOT( updateColorAndAlpha() ));
}

void PointDisplay::onInitialize(){
  frame_node_ = scene_node_->createChildSceneNode();
  vis_arrow_.reset(new rviz::Arrow( scene_manager_, frame_node_));
  float alpha = alpha_property_->getFloat();
  Ogre::ColourValue color = color_property_->getOgreColor();
  vis_arrow_->setColor(color.r, color.g, color.b, alpha);
  Ogre::Vector3 arrow_scale(0, 0, 0);
  vis_arrow_->setScale(arrow_scale);
  MFDClass::onInitialize(); // MFDClass := MessageFilterDisplay<message type>
}

PointDisplay::~PointDisplay(){
}

void PointDisplay::reset(){
  MFDClass::reset();
}

void PointDisplay::updateColorAndAlpha(){
  float alpha = alpha_property_->getFloat();
  Ogre::ColourValue color = color_property_->getOgreColor();
  vis_arrow_->setColor(color.r, color.g, color.b, alpha);
}

void PointDisplay::updateHistoryLength(){
}

void PointDisplay::processMessage(const geometry_msgs::PointStamped::ConstPtr& msg){
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if( !context_->getFrameManager()->getTransform(msg->header.frame_id, msg->header.stamp, position, orientation)){
    ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'", msg->header.frame_id.c_str(), qPrintable( fixed_frame_ ));
    return;
  }
  frame_node_->setPosition(position);
  frame_node_->setOrientation(orientation);
  Ogre::Vector3 arrow_dir(msg->point.x, msg->point.y, msg->point.z);
  float arrow_length = arrow_dir.length() * 0.77;
  Ogre::Vector3 arrow_scale(arrow_length, arrow_length, arrow_length);
  vis_arrow_->setScale(arrow_scale);
  vis_arrow_->setDirection(arrow_dir);
}

} // namespace plugin_lecture

PLUGINLIB_EXPORT_CLASS(plugin_lecture::PointDisplay,rviz::Display )
