#include "qt_render.h"

#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreTextureManager.h>
#include <OGRE/OgreTexture.h>
#include <OGRE/OgreTechnique.h>
#include <OGRE/OgreEntity.h>
#include <OGRE/OgreHardwarePixelBuffer.h>
#include <OGRE/Overlay/OgrePanelOverlayElement.h>
#include <OGRE/Overlay/OgreOverlayElement.h>
#include <OGRE/Overlay/OgreOverlayContainer.h>
#include <OGRE/Overlay/OgreOverlayManager.h>
#include <OGRE/Overlay/OgreTextAreaOverlayElement.h>
#include <QImage>
#include <QColor>
#include <QVBoxLayout>
#include <rviz/uniform_string_stream.h>
#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/mesh_loader.h>

MyRender::MyRender(QWidget* parent) : QDialog(parent)
{
  render_panel_ = new rviz::RenderPanel();
  QVBoxLayout* main_layout = new QVBoxLayout;
  main_layout->addWidget(render_panel_);
  setLayout(main_layout);

  render_panel_->show();

  manager_ = new rviz::VisualizationManager(render_panel_);
  render_panel_->initialize(manager_->getSceneManager(), manager_);
  manager_->initialize();
  manager_->startUpdate();

  scene_manager_ = manager_->getSceneManager();

  // visualize axis
  vis_axes_.reset(new rviz::Axes(scene_manager_));

  // visualize arrow
  vis_arrow_.reset(new rviz::Arrow(scene_manager_));
  Ogre::Vector3 arrow_pos(0, -1, 0);
  vis_arrow_->setPosition(arrow_pos);
  vis_arrow_->setColor(1, 0, 0, 1.0);
  Ogre::Vector3 arrow_dir(0, 0, 1.0);
  float arrow_length = arrow_dir.length();
  Ogre::Vector3 arrow_scale(arrow_length, arrow_length, arrow_length);
  vis_arrow_->setScale(arrow_scale);
  vis_arrow_->setDirection(arrow_dir);

  // visualize line
  vis_line_.reset(new rviz::Line(scene_manager_));
  Ogre::Vector3 start(1, -1, 0);
  Ogre::Vector3 end(1, -1, 1);
  vis_line_->setPoints(start, end);
  vis_line_->setColor(1, 1, 1, 1);

  // visualize billboard line
  vis_billboard_line_.reset(new rviz::BillboardLine(scene_manager_));
  vis_billboard_line_->setLineWidth(0.1);
  vis_billboard_line_->setColor(0, 1, 0, 1);
  Ogre::Vector3 p0(2, -1, 0);
  vis_billboard_line_->addPoint(p0);
  Ogre::Vector3 p1(2, 0, 0);
  vis_billboard_line_->addPoint(p1);
  Ogre::Vector3 p2(2, 0, 1);
  vis_billboard_line_->addPoint(p2);
  Ogre::Vector3 p3(2, 1, 1);
  vis_billboard_line_->addPoint(p3);

  // visualize  cylinder
  vis_shape_.reset(new rviz::Shape(rviz::Shape::Cylinder, scene_manager_));
  Ogre::Vector3 shape_pos(0, 2, 0);
  vis_shape_->setPosition(shape_pos);
  Ogre::Quaternion shape_q(0.7, 0.7, 0, 0);  // (w, x, y, z)
  vis_shape_->setOrientation(shape_q);
  vis_shape_->setColor(0, 0, 1, 1);

  // visualize mesh
  vis_mesh_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  std::string flag_resource = "package://qt_lecture/resources/board.dae";
  if (rviz::loadMeshFromResource(flag_resource).isNull())
  {
    printf("failed to load model resource '%s'.\n", flag_resource.c_str());
    return;
  }
  Ogre::Entity* entity = scene_manager_->createEntity(flag_resource);
  vis_mesh_->attachObject(entity);
  Ogre::Vector3 flag_pos(2, 2, 0);
  vis_mesh_->setPosition(flag_pos);

  // overlay material
  Ogre::Overlay* overlay = Ogre::OverlayManager::getSingleton().create("test03.Overlay");

  int width = 100;
  int height = 100;
  std::string matName = "testmaterial";
  std::string texture_name = "mytexture";
  Ogre::TexturePtr texture = Ogre::TextureManager::getSingleton().createManual(
      texture_name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, Ogre::TEX_TYPE_2D, width, height, 0,
      Ogre::PF_A8R8G8B8, Ogre::TU_DEFAULT);

  Ogre::MaterialPtr mat = (Ogre::MaterialPtr)Ogre::MaterialManager::getSingleton().create(
      matName, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, true);

  mat->getTechnique(0)->getPass(0)->createTextureUnitState(texture_name);
  mat->getTechnique(0)->getPass(0)->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);

  Ogre::HardwarePixelBufferSharedPtr pixel_buffer = texture->getBuffer();
  pixel_buffer->lock(Ogre::HardwareBuffer::HBL_NORMAL);
  const Ogre::PixelBox& pixelBox = pixel_buffer->getCurrentLock();
  Ogre::uint8* pDest = static_cast<Ogre::uint8*>(pixelBox.data);
  memset(pDest, 0, width * height);
  QImage Hud = QImage(pDest, width, height, QImage::Format_ARGB32);
  QColor bg_color(0, 0, 0, 70.0);
  for (unsigned int i = 0; i < width; i++)
  {
    for (unsigned int j = 0; j < height; j++)
    {
      Hud.setPixel(i, j, bg_color.rgba());
    }
  }
  pixel_buffer->unlock();

  Ogre::OverlayElement* e = Ogre::OverlayManager::getSingleton().createOverlayElement("Panel", "TexturePanel");
  e->setMetricsMode(Ogre::GuiMetricsMode::GMM_PIXELS);
  e->setMaterialName(matName);
  e->setDimensions(152, 32);
  e->setPosition(30, 28);
  overlay->add2D((Ogre::OverlayContainer*)e);

  // overlay text
  Ogre::OverlayContainer* panel =
      (Ogre::OverlayContainer*)Ogre::OverlayManager::getSingleton().createOverlayElement("Panel", "TextPanel");
  Ogre::TextAreaOverlayElement* text =
      (Ogre::TextAreaOverlayElement*)Ogre::OverlayManager::getSingleton().createOverlayElement("TextArea", "test03."
                                                                                                           "TextArea");
  text->setMetricsMode(Ogre::GuiMetricsMode::GMM_PIXELS);
  text->setVerticalAlignment(Ogre::GuiVerticalAlignment::GVA_TOP);
  text->setHorizontalAlignment(Ogre::GuiHorizontalAlignment::GHA_LEFT);
  text->setColour(Ogre::ColourValue::White);
  text->setFontName("Liberation Sans");
  text->setCaption("hello world");
  text->setCharHeight(32);
  text->setPosition(32, 32);
  panel->addChild(text);
  overlay->add2D((Ogre::OverlayContainer*)panel);

  overlay->show();
}
