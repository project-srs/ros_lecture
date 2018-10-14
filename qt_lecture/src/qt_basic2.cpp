#include <QApplication>
#include <QPushButton>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>

int main(int argc, char** argv)
{
  QApplication app(argc, argv);

  QWidget* window1 = new QWidget;
  QPushButton* button1A = new QPushButton("Button 1A");
  QPushButton* button1B = new QPushButton("Button 1B");
  QPushButton* button1C = new QPushButton("Button 1C");
  QHBoxLayout* layout1  = new QHBoxLayout;
  layout1->addWidget(button1A);
  layout1->addWidget(button1B);
  layout1->addWidget(button1C);
  window1->setLayout(layout1);
  window1->show();

  QWidget* window2 = new QWidget;
  QPushButton* button2A = new QPushButton("Button 2A");
  QPushButton* button2B = new QPushButton("Button 2B");
  QPushButton* button2C = new QPushButton("Button 2C");
  QVBoxLayout* layout2  = new QVBoxLayout;
  layout2->addWidget(button2A);
  layout2->addWidget(button2B);
  layout2->addWidget(button2C);
  window2->setLayout(layout2);
  window2->show();

  QWidget* window3 = new QWidget;
  QPushButton* button3A = new QPushButton("Button 3A");
  QPushButton* button3B = new QPushButton("Button 3B");
  QPushButton* button3C = new QPushButton("Button 3C");
  QGridLayout* layout3  = new QGridLayout;
  layout3->addWidget(button3A,0,0);
  layout3->addWidget(button3B,0,1);
  layout3->addWidget(button3C,1,0,1,2);
  window3->setLayout(layout3);
  window3->show();

  return app.exec();
}
