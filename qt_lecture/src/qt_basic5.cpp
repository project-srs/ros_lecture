#include <QApplication>
#include <QVBoxLayout>
#include <QLabel>
#include <QCheckBox>

#include "qt_touch.h"

int main(int argc, char** argv)
{
  QApplication app(argc,argv);
  QWidget* window = new QWidget;
  QVBoxLayout* layout  = new QVBoxLayout;

  QCheckBox* check = new QCheckBox("grayout");
  layout->addWidget(check);
  TouchWidget* touch = new TouchWidget(window);
  layout->addWidget(touch);
  QLabel* label = new QLabel("0,0");
  layout->addWidget(label);
  
  QObject::connect(check, SIGNAL(stateChanged(int)), touch, SLOT(checkGrayout(int)));
  QObject::connect(touch, SIGNAL(modifyPosition(QString)), label, SLOT(setText(QString)) );

  window->setLayout(layout);
  window->show();
  return app.exec();
}
