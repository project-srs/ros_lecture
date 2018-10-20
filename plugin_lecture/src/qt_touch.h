#include <QWidget>

class TouchWidget : public QWidget
{
  Q_OBJECT
public:
  TouchWidget(QWidget* parent = 0);
  //property
  bool grayout;
  float x_value;
  float y_value;

  int hcen;
  int vcen;
  int rsize;

  //event
  
  virtual void setEnabled( bool enable );  
  virtual void paintEvent( QPaintEvent* event );
  virtual void mouseMoveEvent( QMouseEvent* event );
  virtual void mousePressEvent( QMouseEvent* event );
  virtual void mouseReleaseEvent( QMouseEvent* event );
  virtual void leaveEvent( QEvent* event );
  void set_value(float x, float y);

public Q_SLOTS:
  void checkGrayout(int state);
Q_SIGNALS:
  void modifyPosition( QString );
};
