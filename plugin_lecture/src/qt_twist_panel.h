#ifndef Q_MOC_RUN
# include <ros/ros.h>
# include <rviz/panel.h>
#endif

#include <QLabel>

class qt_twist_panel: public rviz::Panel
{
Q_OBJECT
public:
  qt_twist_panel( QWidget* parent = 0 );



  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;

/*

  QLineEdit* output_topic_editor_;

  QString output_topic_;

  // The ROS publisher for the command velocity.
  ros::Publisher velocity_publisher_;

  // The ROS node handle.
  ros::NodeHandle nh_;

  float linear_velocity_;
  float angular_velocity_;
*/
};
