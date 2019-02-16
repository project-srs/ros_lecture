#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <std_msgs/String.h>
#endif

#include <QDialog>
#include <QLabel>
#include <QPushButton>
#include <QLineEdit>

class MainDialog : public QDialog
{
  Q_OBJECT
public:
  MainDialog(QWidget* parent);

private:
  QLineEdit* lineEdit;
  ros::NodeHandle nh_;
  ros::Subscriber string_sub_;
  void stringCallback(const std_msgs::String& msg); 
};
