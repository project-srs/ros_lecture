#include <ros/ros.h>
#include <std_msgs/String.h>
#include <functional>

#include <QDialog>
#include <QLabel>
#include <QPushButton>
#include <QLineEdit>
#include <QVBoxLayout>

#include "qt_listener_class.h"

MainDialog::MainDialog(QWidget* parent): QDialog(parent),nh_(){
  lineEdit = new QLineEdit;

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addWidget(lineEdit);
  setLayout(layout);

  string_sub_ = nh_.subscribe("chatter", 10, &MainDialog::stringCallback, this);
  printf("register\n");
}

void MainDialog::stringCallback(const std_msgs::String& string_msg){
  QString text = QString::fromStdString(string_msg.data);
  lineEdit->setText(text);
  ROS_INFO("sub: %s", string_msg.data.c_str());
}
