#include <QGuiApplication>
#include <QQmlApplicationEngine>

int main(int argc, char** argv)
{
  // Init Qt
  QGuiApplication app(argc, argv);

  if (argc < 2)
  {
    printf("qml filename is ewquired.\n");
    return 0;
  }

  QQmlApplicationEngine engine(&app);
  engine.load(QUrl(argv[1]));

  return app.exec();
}
