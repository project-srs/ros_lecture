#include "qt_render.h"

#include <QApplication>
#include <QLabel>
#include <ros/ros.h>

int main(int argc, char **argv){
  ros::init(argc, argv, "qt_renader");
  QApplication app( argc, argv );
  MyRender* my_render = new MyRender();
  my_render->show();
  return app.exec();
}
