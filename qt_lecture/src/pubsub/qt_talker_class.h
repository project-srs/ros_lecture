#ifndef Q_MOC_RUN
#include <ros/ros.h>
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

private Q_SLOTS:
  void publishString(); 
private:
  QPushButton* setButton;
  ros::NodeHandle nh_;
  ros::Publisher string_pub_; 
};
