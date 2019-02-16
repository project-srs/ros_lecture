#include <string>
#include <sstream>
#include <iostream>
#include <QPainter>
#include <QMouseEvent>
#include <QSizePolicy>

#include "qt_touch.h"

TouchWidget::TouchWidget(QWidget* parent): QWidget( parent )
{
  setMinimumSize(100,100);
  setSizePolicy(QSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding));
  grayout=false;
}

void TouchWidget::paintEvent( QPaintEvent* event )
{
  int w = width();
  int h = height();
  int size = (( w > h ) ? h : w) - 1;
  int hpad = ( w - size ) / 2;
  int vpad = ( h - size ) / 2;
  hcen = hpad + size/2;
  vcen = vpad + size/2;
  rsize=size/2;

  QColor background;
  QColor crosshair;
  if(!grayout){
    background = Qt::white;
    crosshair  = Qt::black;;
  }
  else{
    background = Qt::lightGray;
    crosshair = Qt::darkGray;
  }
  QPainter painter( this );
  painter.setBrush( background );
  painter.setPen( crosshair );

  painter.drawRect( QRect( hpad, vpad, size, size ));
  int rline=size/2;
  painter.drawLine( hcen, vcen-rline, hcen, vcen+rline );
  painter.drawLine( hcen-rline, vcen, hcen+rline, vcen );

  if(!grayout){
    QPen arrow;
    arrow.setWidth( size/20 );
    arrow.setColor( Qt::black );
    arrow.setCapStyle( Qt::RoundCap );
    arrow.setJoinStyle( Qt::RoundJoin );
    painter.setPen( arrow );

    const int step_count = 2;
    QPointF arrows[ step_count ];
    arrows[0].setX(hcen);
    arrows[0].setY(vcen);
    arrows[1].setX((int)(hcen+x_value*rsize));
    arrows[1].setY((int)(vcen+y_value*rsize));
    painter.drawPolyline( arrows, 2 );
  }
}

void TouchWidget::checkGrayout(int state){
  if(state>0)grayout=true;
  else grayout=false;
  update();
} 


void TouchWidget::mouseMoveEvent( QMouseEvent* event )
{
  float tmp_x=1.0*(event->x()-hcen)/rsize;
  float tmp_y=1.0*(event->y()-vcen)/rsize;
  if(tmp_x<-1.0)tmp_x=-1.0;
  else if(tmp_x>1.0)tmp_x=1.0;
  if(tmp_y<-1.0)tmp_y=-1.0;
  else if(tmp_y>1.0)tmp_y=1.0;
  set_value(tmp_x, tmp_y);
  update();
}
void TouchWidget::mousePressEvent( QMouseEvent* event )
{
  float tmp_x=1.0*(event->x()-hcen)/rsize;
  float tmp_y=1.0*(event->y()-vcen)/rsize;
  if(tmp_x<-1.0)tmp_x=-1.0;
  else if(tmp_x>1.0)tmp_x=1.0;
  if(tmp_y<-1.0)tmp_y=-1.0;
  else if(tmp_y>1.0)tmp_y=1.0;
  set_value(tmp_x, tmp_y);
  update();
}
void TouchWidget::leaveEvent( QEvent* event )
{
  set_value(0, 0);
  update();
}
void TouchWidget::mouseReleaseEvent( QMouseEvent* event )
{
  set_value(0, 0);
  update();
}
void TouchWidget::set_value(float x, float y){
  if(!grayout){
    x_value=x;
    y_value=y;
    std::ostringstream stm ;
    stm << x << ","<<y;
    QString text=QString::fromStdString(stm.str());
    Q_EMIT modifyPosition(text);
  }
}
