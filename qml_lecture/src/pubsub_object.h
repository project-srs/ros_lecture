#pragma once

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <QObject>
#include <QVariant>
#include <QDebug>

class RosStringObject : public QObject
{
  Q_OBJECT
public:
  RosStringObject(QObject* parent = nullptr)
  {
    connect(this, &RosStringObject::publishString, this, &RosStringObject::publishStringSlot);
    string_pub_ = nh_.advertise<std_msgs::String>("chatter", 10);
    string_sub_ = nh_.subscribe("chatter", 10, &RosStringObject::stringCallback, this);
  }

signals:
  void publishString(QString s);
  void subscribeString(QString text);

private slots:
  void publishStringSlot(QString s)
  {
    std_msgs::String msg;
    msg.data = s.toStdString();
    string_pub_.publish(msg);
  }

private:
  void stringCallback(const std_msgs::String& msg)
  {
    emit subscribeString(QString(msg.data.c_str()));
  }

  ros::NodeHandle nh_;
  ros::Publisher string_pub_;
  ros::Subscriber string_sub_;
};
