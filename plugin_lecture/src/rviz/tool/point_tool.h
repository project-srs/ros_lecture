#ifndef SET_MARKER_H
#define SET_MARKER_H

#include <ros/ros.h>
#include <rviz/tool.h>
#include <rviz/ogre_helpers/shape.h>

namespace Ogre{
class SceneNode;
class Vector3;
}

namespace rviz{
class VectorProperty;
class VisualizationManager;
class ViewportMouseEvent;
}

namespace plugin_lecture{

class PointTool: public rviz::Tool {
Q_OBJECT
public:
  PointTool();
  ~PointTool();

  virtual void onInitialize();
  virtual void activate();
  virtual void deactivate();
  virtual int processMouseEvent( rviz::ViewportMouseEvent& event );

  ros::NodeHandle nh_;
  ros::Publisher point_pub_;
  boost::shared_ptr<rviz::Shape> vis_shape_;
};

} // namespace plugin_lecture

#endif // SET_MARKER_H
