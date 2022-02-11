#include <QApplication>
#include <QVBoxLayout>
#include <QPushButton>
#include <QLineEdit>
#include <QLabel>

int main(int argc, char** argv)
{
  QApplication app(argc, argv);

  QWidget* window = new QWidget;
  QVBoxLayout* layout  = new QVBoxLayout;
  QPushButton* button  = new QPushButton("Quit");
  QLineEdit*   edit    = new QLineEdit("");
  QLabel*      label   = new QLabel("");

  layout->addWidget(button);
  layout->addWidget(edit);
  layout->addWidget(label);
  window->setLayout(layout);

  QObject::connect(button, &QPushButton::clicked, &app, &QApplication::quit);
  QObject::connect(edit, &QLineEdit::textChanged, label, &QLabel::setText);

  window->show();

  return app.exec();
}
