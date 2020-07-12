#include "log_display.h"
#include <pluginlib/class_list_macros.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreTextureManager.h>
#include <OGRE/OgreTexture.h>
#include <OGRE/OgreTechnique.h>
#include <OGRE/OgreHardwarePixelBuffer.h>
#include <OGRE/Overlay/OgrePanelOverlayElement.h>
#include <OGRE/Overlay/OgreOverlayElement.h>
#include <OGRE/Overlay/OgreOverlayContainer.h>
#include <OGRE/Overlay/OgreOverlayManager.h>
#include <QImage>
#include <QColor>

namespace plugin_lecture
{
LogDisplay::LogDisplay()
{
  height_property_ = new rviz::IntProperty("height", 200, "window height", this, SLOT(renderOverlay()));
  width_property_ = new rviz::IntProperty("width", 400, "window width", this, SLOT(renderOverlay()));
  size_property_ = new rviz::IntProperty("size", 16, "window size", this, SLOT(renderOverlay()));

  overlay_ = Ogre::OverlayManager::getSingleton().create("RvizLogOverlay");
  // overlay text
  panel_ = (Ogre::OverlayContainer*)Ogre::OverlayManager::getSingleton().createOverlayElement("Panel", "TextPanel");
  overlay_->add2D(panel_);

  // overlay material
  createMaterial("BackGround");
  mat_element_ = Ogre::OverlayManager::getSingleton().createOverlayElement("Panel", "TexturePanel");
  mat_element_->setMetricsMode(Ogre::GuiMetricsMode::GMM_PIXELS);
  mat_element_->setMaterialName("BackGround");
  overlay_->add2D((Ogre::OverlayContainer*)mat_element_);

  overlay_->show();

  log_sub_ = nh_.subscribe("/rosout", 10, &LogDisplay::processMessage, this);
}

LogDisplay::~LogDisplay()
{
  Ogre::OverlayManager::getSingleton().destroy("RvizLogOverlay");
  // text
  Ogre::OverlayManager::getSingleton().destroyOverlayElement(panel_);
  for (auto element : text_elements_)
    Ogre::OverlayManager::getSingleton().destroyOverlayElement(element);
  // material
  Ogre::OverlayManager::getSingleton().destroyOverlayElement(mat_element_);
  destroyMaterial("BackGround");
}

void LogDisplay::save(rviz::Config config) const
{
  rviz::Display::save(config);
  config.mapSetValue("Height", height_property_->getInt());
  config.mapSetValue("Width", width_property_->getInt());
  config.mapSetValue("Size", size_property_->getInt());
}

void LogDisplay::load(const rviz::Config& config)
{
  rviz::Display::load(config);
  int temp_int;
  if (config.mapGetInt("Height", &temp_int))
    height_property_->setInt(temp_int);
  if (config.mapGetInt("Width", &temp_int))
    width_property_->setInt(temp_int);
  if (config.mapGetInt("Size", &temp_int))
    size_property_->setInt(temp_int);
}

Ogre::TextAreaOverlayElement* LogDisplay::createTextElement(int index)
{
  std::string name = "Text" + std::to_string(index);
  Ogre::TextAreaOverlayElement* text_ptr =
      (Ogre::TextAreaOverlayElement*)Ogre::OverlayManager::getSingleton().createOverlayElement("TextArea", name);
  text_ptr->setMetricsMode(Ogre::GuiMetricsMode::GMM_PIXELS);
  text_ptr->setVerticalAlignment(Ogre::GuiVerticalAlignment::GVA_TOP);
  text_ptr->setHorizontalAlignment(Ogre::GuiHorizontalAlignment::GHA_LEFT);
  text_ptr->setColour(Ogre::ColourValue::White);
  text_ptr->setFontName("Liberation Sans");
  text_ptr->setCaption("hello world");
  text_ptr->setCharHeight(16);
  text_ptr->setPosition(16, 16 * (index + 1));
  return text_ptr;
}

void LogDisplay::processMessage(const rosgraph_msgs::Log& msg)
{
  if (msg.level == rosgraph_msgs::Log::DEBUG)
    return;

  log_msgs_.push_back(msg);
  int max_lines = height_property_->getInt() / size_property_->getInt();
  while (max_lines < log_msgs_.size())
    log_msgs_.pop_front();

  renderOverlay();
}

void LogDisplay::createMaterial(std::string mat_name)
{
  int width = 100;
  int height = 100;
  std::string texture_name = mat_name + "Texture";
  QColor bg_color(0, 0, 0, 70.0);

  Ogre::TexturePtr texture = Ogre::TextureManager::getSingleton().createManual(
      texture_name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, Ogre::TEX_TYPE_2D, width, height, 0,
      Ogre::PF_A8R8G8B8, Ogre::TU_DEFAULT);

  Ogre::MaterialPtr mat = (Ogre::MaterialPtr)Ogre::MaterialManager::getSingleton().create(
      mat_name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, true);

  mat->getTechnique(0)->getPass(0)->createTextureUnitState(texture_name);
  mat->getTechnique(0)->getPass(0)->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);

  Ogre::HardwarePixelBufferSharedPtr pixel_buffer = texture->getBuffer();
  pixel_buffer->lock(Ogre::HardwareBuffer::HBL_NORMAL);
  const Ogre::PixelBox& pixelBox = pixel_buffer->getCurrentLock();
  Ogre::uint8* pDest = static_cast<Ogre::uint8*>(pixelBox.data);
  memset(pDest, 0, width * height);
  QImage Hud = QImage(pDest, width, height, QImage::Format_ARGB32);
  for (unsigned int i = 0; i < width; i++)
  {
    for (unsigned int j = 0; j < height; j++)
    {
      Hud.setPixel(i, j, bg_color.rgba());
    }
  }
  pixel_buffer->unlock();
}

void LogDisplay::destroyMaterial(std::string mat_name)
{
  std::string texture_name = mat_name + "Texture";
  Ogre::TextureManager::getSingleton().destroyResourcePool(texture_name);
  Ogre::MaterialManager::getSingleton().destroyResourcePool(mat_name);
}

void LogDisplay::renderOverlay()
{
  if (!isEnabled())
  {
    overlay_->hide();
    return;
  }
  overlay_->show();

  int view_width = Ogre::OverlayManager::getSingleton().getViewportWidth();
  int window_width = width_property_->getInt();
  int window_height = height_property_->getInt();
  int text_size = size_property_->getInt();

  mat_element_->setDimensions(window_width, window_height);
  mat_element_->setPosition(view_width - window_width, 0);

  while (text_elements_.size() < log_msgs_.size())
  {
    int id = text_elements_.size();
    text_elements_.push_back(createTextElement(id));
    panel_->addChild(text_elements_.back());
  }
  for (int i = log_msgs_.size(); i < text_elements_.size(); i++)
  {
    text_elements_[i]->setCaption("");
  }

  for (int i = 0; i < log_msgs_.size(); i++)
  {
    if (log_msgs_[i].level == rosgraph_msgs::Log::FATAL || log_msgs_[i].level == rosgraph_msgs::Log::ERROR)
      text_elements_[i]->setColour(Ogre::ColourValue::Red);
    else if (log_msgs_[i].level == rosgraph_msgs::Log::WARN)
      text_elements_[i]->setColour(Ogre::ColourValue(1, 1, 0));
    else
      text_elements_[i]->setColour(Ogre::ColourValue::White);

    text_elements_[i]->setPosition(view_width - window_width, text_size * i);
    text_elements_[i]->setCharHeight(text_size);

    std::string text = "[" + log_msgs_[i].name + "] " + log_msgs_[i].msg;
    text_elements_[i]->setCaption(text);
  }
}

}  // namespace plugin_lecture

PLUGINLIB_EXPORT_CLASS(plugin_lecture::LogDisplay, rviz::Display)
