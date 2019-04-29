#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <pluginlib/class_list_macros.h>

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
#include "qt_twist_panel.h"

namespace plugin_lecture{
qt_twist_panel::qt_twist_panel( QWidget* parent )
  : rviz::Panel( parent )
{
  QVBoxLayout* layout = new QVBoxLayout;

  QHBoxLayout* layout_1st = new QHBoxLayout;
  enable_check_ = new QCheckBox("Enable");
  layout_1st->addWidget(enable_check_);
  layout_1st->addWidget( new QLabel("Topic:"));
  topic_edit_ = new QLineEdit("");
  layout_1st->addWidget(topic_edit_);
  layout->addLayout(layout_1st);

  QHBoxLayout* layout_2nd = new QHBoxLayout;
  stamped_check_ = new QCheckBox("Stamped");
  layout_2nd->addWidget(stamped_check_);
  layout_2nd->addWidget( new QLabel("Frame:"));
  frame_edit_ = new QLineEdit("");
  layout_2nd->addWidget(frame_edit_);
  layout->addLayout(layout_2nd);

  QHBoxLayout* layout_3rd = new QHBoxLayout;
  radio1_ =new QRadioButton("X-Y");
  layout_3rd->addWidget(radio1_);
  layout_3rd->addWidget( new QLabel("X max:"));
  max1_edit_ = new QLineEdit("");
  layout_3rd->addWidget(max1_edit_);
  layout_3rd->addWidget( new QLabel("Y max:"));
  max2_edit_ = new QLineEdit("");
  layout_3rd->addWidget(max2_edit_);
  layout->addLayout(layout_3rd);

  QHBoxLayout* layout_4th = new QHBoxLayout;
  radio2_ =new QRadioButton("X-Yaw");
  layout_4th->addWidget(radio2_);
  layout_4th->addWidget( new QLabel("Yaw max:"));
  max3_edit_ = new QLineEdit("");
  layout_4th->addWidget(max3_edit_);
  layout->addLayout(layout_4th);
  
  QButtonGroup *group1=new QButtonGroup();
  group1->addButton(radio1_);
  group1->addButton(radio2_);

  touch_ = new TouchWidget();
  layout->addWidget(touch_);

  setLayout(layout);

  QTimer* output_timer = new QTimer(this);
  connect( output_timer, SIGNAL(timeout()), this, SLOT(tick()));
  output_timer->start(100);

  touch_->setEnabled(false);
  touch_->update();
}
qt_twist_panel::~qt_twist_panel(){
  if(twist_publisher_){
    twist_publisher_.shutdown();
  }
}

void qt_twist_panel::tick()
{
  if( ros::ok())
  {
    if(enable_check_->isChecked()){
      if(twist_publisher_){
        float vel_max1=max1_edit_->text().toFloat();
        float vel_max2=max2_edit_->text().toFloat();
        float vel_max3=max3_edit_->text().toFloat();

        geometry_msgs::TwistStamped msg;
        msg.header.frame_id=pub_frame_;
        msg.header.stamp=ros::Time::now();
        if(radio1_->isChecked()){
          msg.twist.linear.x = -1 * vel_max1 * (touch_->y_value);
          msg.twist.linear.y = -1 * vel_max2 * (touch_->x_value);
        }
        else if(radio2_->isChecked()){
          msg.twist.linear.x  = -1 * vel_max1 * (touch_->y_value);
          msg.twist.angular.z = -1 * vel_max3 * (touch_->x_value);          
        }
        if(pub_stamped_)twist_publisher_.publish(msg);
        else twist_publisher_.publish(msg.twist);
      }
      else{
        std::string topic_name=topic_edit_->text().toStdString();
        if(topic_name!=""){
          if(stamped_check_->isChecked()){
            std::string frame_name=frame_edit_->text().toStdString();
            if(frame_name!=""){
              twist_publisher_ = nh_.advertise<geometry_msgs::TwistStamped>(topic_name, 10 );
              pub_stamped_=true;
              pub_frame_=frame_name;
              //gray to process
              topic_edit_->setEnabled(false);
              stamped_check_->setEnabled(false);
              frame_edit_->setEnabled(false);
              touch_->setEnabled(true);
            }
          }
          else{
            twist_publisher_ = nh_.advertise<geometry_msgs::Twist>(topic_name, 10 );
            pub_stamped_=false;
            //gray to process
            topic_edit_->setEnabled(false);
            stamped_check_->setEnabled(false);
            frame_edit_->setEnabled(false);
            touch_->setEnabled(true);
          }
        }
      }
    }
    else{//Not checked
      if(twist_publisher_){
        twist_publisher_.shutdown();
        //gray to not process
        topic_edit_->setEnabled(true);
        stamped_check_->setEnabled(true);
        frame_edit_->setEnabled(true);
        touch_->setEnabled(false);
      }
    }
  }
}

void qt_twist_panel::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
  config.mapSetValue( "Topic", topic_edit_->text());
  config.mapSetValue( "Stamped", stamped_check_->isChecked());
  config.mapSetValue( "Frame", frame_edit_->text());
  config.mapSetValue( "radio1", radio1_->isChecked());
  config.mapSetValue( "radio2", radio2_->isChecked());
  config.mapSetValue( "max1", max1_edit_->text());
  config.mapSetValue( "max2", max2_edit_->text());
  config.mapSetValue( "max3", max3_edit_->text());
}

void qt_twist_panel::load( const rviz::Config& config )
{
  rviz::Panel::load( config );

  QString tmp_text;
  bool tmp_bool;
  if( config.mapGetString( "Topic", &tmp_text ))topic_edit_->setText( tmp_text );
  if( config.mapGetBool( "Stamped", &tmp_bool ))stamped_check_->setChecked( tmp_bool );
  if( config.mapGetString( "Frame", &tmp_text ))frame_edit_->setText( tmp_text );
  if( config.mapGetBool( "radio1", &tmp_bool ))radio1_->setChecked( tmp_bool );
  if( config.mapGetBool( "radio2", &tmp_bool ))radio2_->setChecked( tmp_bool );
  if( config.mapGetString( "max1", &tmp_text ))max1_edit_->setText( tmp_text );
  if( config.mapGetString( "max2", &tmp_text ))max2_edit_->setText( tmp_text );
  if( config.mapGetString( "max3", &tmp_text ))max3_edit_->setText( tmp_text );  
}
}
PLUGINLIB_EXPORT_CLASS(plugin_lecture::qt_twist_panel,rviz::Panel )
