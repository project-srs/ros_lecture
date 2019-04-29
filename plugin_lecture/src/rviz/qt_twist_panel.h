#ifndef Q_MOC_RUN
# include <ros/ros.h>
# include <rviz/panel.h>
#endif

#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QCheckBox>
#include <QLineEdit>
#include <QRadioButton>
#include <QButtonGroup>
#include <QTimer>

#include <string>

namespace plugin_lecture{
class qt_twist_panel: public rviz::Panel{
Q_OBJECT
public:
  qt_twist_panel( QWidget* parent = 0 );
  ~qt_twist_panel();

  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;

public Q_SLOTS:
  void tick();

public:
  // The ROS node handle.
  ros::NodeHandle nh_;
  // The ROS publisher for the command velocity.
  ros::Publisher twist_publisher_;

  QCheckBox* enable_check_;
  QLineEdit* topic_edit_;

  QCheckBox* stamped_check_;
  QLineEdit* frame_edit_;

  QRadioButton* radio1_;
  QRadioButton* radio2_;

  QLineEdit* max1_edit_;
  QLineEdit* max2_edit_;
  QLineEdit* max3_edit_;

  TouchWidget* touch_;

  bool pub_stamped_;
  std::string pub_frame_;
};
}