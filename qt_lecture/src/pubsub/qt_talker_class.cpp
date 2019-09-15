#include <ros/ros.h>
#include <std_msgs/String.h>

#include <QDialog>
#include <QLabel>
#include <QPushButton>
#include <QLineEdit>
#include <QVBoxLayout>

#include "qt_talker_class.h"

MainDialog::MainDialog(QWidget* parent): QDialog(parent),nh_(){
  setButton = new QPushButton("publish");

  connect(setButton,SIGNAL(clicked()),this,SLOT(publishString()));

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addWidget(setButton);
  setLayout(layout);

  string_pub_ = nh_.advertise<std_msgs::String>("chatter", 10);
}

void MainDialog::publishString(){
  std_msgs::String string_msg;
  string_msg.data="string";
  string_pub_.publish(string_msg);
  ROS_INFO("pub: %s", string_msg.data.c_str()); 
}
