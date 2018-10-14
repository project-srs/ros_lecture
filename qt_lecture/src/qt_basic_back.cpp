#include <QDialog>
#include <QLabel>
#include <QPushButton>
#include <QLineEdit>
#include <QHBoxLayout>
#include <QApplication>
#include <QtGui>

#include "qt_lib1.h"

/*
MainDialog::MainDialog(QWidget* parent)
	: QDialog(parent){}
void MainDialog::setLabelText(){}
*/
int main(int argc, char** argv)
{
	printf("######\n%s\n#######\n",QT_VERSION_STR);
	QApplication app(argc,argv);
	QWidget* window = new QWidget;
	MainDialog* dialog = new MainDialog(window);
	//QDialog* dialog = new QDialog(window);
	dialog->show();
	return app.exec();
}
