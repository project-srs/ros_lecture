#include <QDialog>
#include <QLabel>
#include <QPushButton>
#include <QLineEdit>
#include <QVBoxLayout>

#include "qt_mydialog.h"

MainDialog::MainDialog(QWidget* parent): QDialog(parent)
{
  label = new QLabel(tr("empty") );
  setButton = new QPushButton(tr("Set") );
  lineEdit = new QLineEdit;

  connect(setButton,SIGNAL(clicked()),this,SLOT(setLabelText()));

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addWidget(label);
  layout->addWidget(lineEdit);
  layout->addWidget(setButton);
  setLayout(layout);
}

void MainDialog::setLabelText()
{
  QString text = lineEdit->text();
  label->setText(text);
}
