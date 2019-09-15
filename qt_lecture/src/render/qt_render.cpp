#include "qt_render.h"

#include <QDialog>
#include <QLabel>
#include <QPushButton>
#include <QLineEdit>
#include <QVBoxLayout>

#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display.h>

MyRender::MyRender(QWidget* parent) : QDialog(parent) {
  render_panel_ = new rviz::RenderPanel();
  QVBoxLayout* main_layout = new QVBoxLayout;
  main_layout->addWidget( render_panel_ );
  setLayout( main_layout );

  manager_ = new rviz::VisualizationManager( render_panel_ );
  render_panel_->initialize( manager_->getSceneManager(), manager_ );
  manager_->initialize();
  manager_->startUpdate();

  scene_manager_ = manager_->getSceneManager();

  // visualize axis
  vis_axes_.reset(new rviz::Axes( scene_manager_));

  // visualize arrow
  vis_arrow_.reset(new rviz::Arrow( scene_manager_));
  Ogre::Vector3 arrow_pos(0, -1, 0);
  vis_arrow_->setPosition(arrow_pos);
  vis_arrow_->setColor(1, 0, 0, 1.0);
  Ogre::Vector3 arrow_dir(0, 0, 1.0);
  float arrow_length = arrow_dir.length();
  Ogre::Vector3 arrow_scale(arrow_length, arrow_length, arrow_length);
  vis_arrow_->setScale(arrow_scale);
  vis_arrow_->setDirection(arrow_dir);

  // visualize line
  vis_line_.reset(new rviz::Line( scene_manager_));
  Ogre::Vector3 start(1, -1, 0);
  Ogre::Vector3 end(1, -1, 1);
  vis_line_->setPoints(start, end);
  vis_line_->setColor(1, 1, 1, 1);

  // visualize billboard line
  vis_billboard_line_.reset(new rviz::BillboardLine( scene_manager_));
  vis_billboard_line_->setLineWidth (0.1);
  vis_billboard_line_->setColor(0, 1, 0, 1);
  Ogre::Vector3 p0(2, -1, 0);
  vis_billboard_line_->addPoint(p0);
  Ogre::Vector3 p1(2, 0, 0);
  vis_billboard_line_->addPoint(p1);
  Ogre::Vector3 p2(2, 0, 1);
  vis_billboard_line_->addPoint(p2);
  Ogre::Vector3 p3(2, 1, 1);
  vis_billboard_line_->addPoint(p3);

  // visualize shape
  vis_shape_.reset(new rviz::Shape(rviz::Shape::Cylinder, scene_manager_));
  Ogre::Vector3 shape_pos(0,2,0);
  vis_shape_->setPosition(shape_pos); 
  Ogre::Quaternion  shape_q(0.7, 0.7, 0, 0); // (w, x, y, z)
  vis_shape_->setOrientation(shape_q);   
  vis_shape_->setColor(0, 0, 1, 1);

  // visualize mesh
  vis_mesh_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  std::string flag_resource = "package://qt_lecture/resources/board.dae";
  if( rviz::loadMeshFromResource( flag_resource ).isNull() ){
    ROS_ERROR( "failed to load model resource '%s'.", flag_resource.c_str());
    return;
  }
  Ogre::Entity* entity = scene_manager_->createEntity( flag_resource );
  vis_mesh_->attachObject( entity );
  Ogre::Vector3 flag_pos(2, 2, 0);
  vis_mesh_->setPosition(flag_pos); 
}
