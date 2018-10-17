#include <stdio.h>

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QCheckBox>
#include <QLineEdit>
#include <QGroupBox>
#include <QRadioButton>
#include <QButtonGroup>

#include "qt_touch.h"

#include <geometry_msgs/Twist.h>

#include "qt_twist_panel.h"

qt_twist_panel::qt_twist_panel( QWidget* parent )
  : rviz::Panel( parent )
{
  QVBoxLayout* layout = new QVBoxLayout;

  QHBoxLayout* layout_1st = new QHBoxLayout;
  QCheckBox* enable_check = new QCheckBox("Enable");
  layout_1st->addWidget(enable_check);
  layout_1st->addWidget( new QLabel("Topic:"));
  QLineEdit* topic_edit = new QLineEdit("");
  layout_1st->addWidget(topic_edit);
  layout->addLayout(layout_1st);

  QHBoxLayout* layout_2nd = new QHBoxLayout;
  QCheckBox* stamped_check = new QCheckBox("Stamped");
  layout_2nd->addWidget(stamped_check);
  layout_2nd->addWidget( new QLabel("Frame:"));
  QLineEdit* frame_edit = new QLineEdit("");
  layout_2nd->addWidget(frame_edit);
  layout->addLayout(layout_2nd);

  QHBoxLayout* layout_3rd = new QHBoxLayout;
  QRadioButton* radio1 =new QRadioButton("X-Y");
  layout_3rd->addWidget(radio1);
  QRadioButton* radio2 =new QRadioButton("X-Yaw");
  layout_3rd->addWidget(radio2);
  QButtonGroup *group1=new QButtonGroup();
  group1->addButton(radio1);
  group1->addButton(radio2);
  layout->addLayout(layout_3rd);

  QHBoxLayout* layout_4th = new QHBoxLayout;
  layout_4th->addWidget( new QLabel("X max:"));
  QLineEdit* max1_edit = new QLineEdit("");
  layout_4th->addWidget(max1_edit);
  layout->addLayout(layout_4th);
  
  QHBoxLayout* layout_5th = new QHBoxLayout;
  layout_5th->addWidget( new QLabel("Y max:"));
  QLineEdit* max2_edit = new QLineEdit("");
  layout_5th->addWidget(max2_edit);
  layout->addLayout(layout_5th);

  QHBoxLayout* layout_6th = new QHBoxLayout;
  layout_6th->addWidget( new QLabel("Yaw max:"));
  QLineEdit* max3_edit = new QLineEdit("");
  layout_6th->addWidget(max3_edit);
  layout->addLayout(layout_6th);

  TouchWidget* touch = new TouchWidget();
  layout->addWidget(touch);

  setLayout( layout );

/*
  QTimer* output_timer = new QTimer( this );
  connect( output_timer, SIGNAL( timeout() ), this, SLOT( sendVel() ));
  output_timer->start( 100 );
*/
}
/*
void TeleopPanel::sendVel()
{
  if( ros::ok() && velocity_publisher_ )
  {
    geometry_msgs::Twist msg;
    msg.linear.x = linear_velocity_;
    msg.linear.y = 0;
    msg.linear.z = 0;
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = angular_velocity_;
    velocity_publisher_.publish( msg );
  }
}
*/

void qt_twist_panel::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
  //config.mapSetValue( "Topic", output_topic_ );
}

void qt_twist_panel::load( const rviz::Config& config )
{
  rviz::Panel::load( config );
/*
  QString topic;
  if( config.mapGetString( "Topic", &topic ))
  {
    output_topic_editor_->setText( topic );
    updateTopic();
  }
*/
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(qt_twist_panel,rviz::Panel )
