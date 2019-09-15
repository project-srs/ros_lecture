#include <QApplication>
#include <QDialog>

#include "qt_mydialog.h"

int main(int argc, char** argv)
{
	QApplication app(argc,argv);
	MainDialog* dialog = new MainDialog();
	dialog->show();
	return app.exec();
}
