#ifndef POINT_DISPLAY_H
#define POINT_DISPLAY_H

#ifndef Q_MOC_RUN
#include <boost/circular_buffer.hpp>
#include <rviz/message_filter_display.h>
#include <rviz/ogre_helpers/arrow.h>
#include <geometry_msgs/PointStamped.h>
#endif

namespace Ogre{
class SceneNode;
}

namespace rviz{
class ColorProperty;
class FloatProperty;
class IntProperty;
}

namespace plugin_lecture{

class PointDisplay: public rviz::MessageFilterDisplay<geometry_msgs::PointStamped>{
Q_OBJECT
public:
  PointDisplay();
  virtual ~PointDisplay();

  virtual void onInitialize();
  virtual void reset();

private Q_SLOTS:
  void updateColorAndAlpha();
  void updateHistoryLength();

private:
  void processMessage( const geometry_msgs::PointStamped::ConstPtr& msg );
  Ogre::SceneNode* frame_node_;
  boost::shared_ptr<rviz::Arrow> vis_arrow_;
  rviz::ColorProperty* color_property_;
  rviz::FloatProperty* alpha_property_;
};

} // namespace plugin_lecture

#endif // POINT_DISPLAY_H
