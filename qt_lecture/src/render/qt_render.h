#include <QDialog>

#ifndef Q_MOC_RUN
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <rviz/ogre_helpers/arrow.h>
#include <rviz/ogre_helpers/axes.h>
#include <rviz/ogre_helpers/line.h>
#include <rviz/ogre_helpers/billboard_line.h>
#include <rviz/ogre_helpers/shape.h>
#endif

namespace rviz
{
class RenderPanel;
class VisualizationManager;
}  // namespace rviz

class MyRender : public QDialog
{
  Q_OBJECT
public:
  MyRender(QWidget* parent = 0);

  rviz::VisualizationManager* manager_;
  rviz::RenderPanel* render_panel_;

  Ogre::SceneNode* frame_node_;
  Ogre::SceneManager* scene_manager_;

  boost::shared_ptr<rviz::Axes> vis_axes_;
  boost::shared_ptr<rviz::Arrow> vis_arrow_;
  boost::shared_ptr<rviz::Line> vis_line_;
  boost::shared_ptr<rviz::BillboardLine> vis_billboard_line_;
  boost::shared_ptr<rviz::Shape> vis_shape_;
  Ogre::SceneNode* vis_mesh_;
};
