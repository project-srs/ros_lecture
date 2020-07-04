#include <ros/ros.h>
#include <std_msgs/String.h>

#include <QCoreApplication>
#include <QObject>
#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QUrl>
#include <QString>
#include <QtQml>
#include <QtConcurrent/QtConcurrent>
#include <QFuture>
#include <QFutureWatcher>

#include "pubsub_object.h"

int main(int argc, char** argv)
{
  if (argc < 2)
  {
    printf("qml filename is ewquired.\n");
    return 0;
  }

  ros::init(argc, argv, "qml_ros", ros::init_options::AnonymousName);
  ros::NodeHandle nh;

  QGuiApplication app(argc, argv);
  RosStringObject ros_string(&app);

  // for ros spin()
  QFutureWatcher<void> rosThread;
  rosThread.setFuture(QtConcurrent::run(&ros::spin));
  QObject::connect(&rosThread, &QFutureWatcher<void>::finished, &app, &QCoreApplication::quit);
  QObject::connect(&app, &QCoreApplication::aboutToQuit, []() { ros::shutdown(); });

  QQmlApplicationEngine engine(&app);
  engine.rootContext()->setContextProperty("ros_string", &ros_string);
  engine.load(QUrl(argv[1]));

  return app.exec();
}
