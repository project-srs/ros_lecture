#include <ros/ros.h>

#include <QApplication>
#include <QDialog>

#include "qt_talker_class.h"

int main(int argc, char** argv){
  ros::init(argc, argv, "qt_talker");
	QApplication app(argc,argv);
	QWidget* window = new QWidget;
	MainDialog* dialog = new MainDialog(window);
	dialog->show();

	ros::Rate loop_rate(20); 
	while (ros::ok()){
		ros::spinOnce();
    app.processEvents();
		loop_rate.sleep();
  }
}
