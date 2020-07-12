#ifndef LOG_DISPLAY_H
#define LOG_DISPLAY_H

#ifndef Q_MOC_RUN
#include <rviz/display.h>
#include <rviz/properties/int_property.h>
#include <rosgraph_msgs/Log.h>
#include <OGRE/Overlay/OgreTextAreaOverlayElement.h>
#endif

namespace plugin_lecture
{
class LogDisplay : public rviz::Display
{
  Q_OBJECT
public:
  LogDisplay();
  virtual ~LogDisplay();
  virtual void load(const rviz::Config& config);
  virtual void save(rviz::Config config) const;

private Q_SLOTS:
  void renderOverlay();

private:
  void processMessage(const rosgraph_msgs::Log& msg);
  rviz::IntProperty* height_property_;
  rviz::IntProperty* width_property_;
  rviz::IntProperty* size_property_;

  Ogre::TextAreaOverlayElement* createTextElement(int index);
  void createMaterial(std::string mat_name);
  void destroyMaterial(std::string mat_name);

  ros::NodeHandle nh_;
  ros::Subscriber log_sub_;
  std::deque<rosgraph_msgs::Log> log_msgs_;

  Ogre::Overlay* overlay_;
  Ogre::OverlayContainer* panel_;
  std::vector<Ogre::TextAreaOverlayElement*> text_elements_;
  Ogre::OverlayElement* mat_element_;
};

}  // namespace plugin_lecture

#endif  // LOG_DISPLAY_H
