#include <future>
#include <unistd.h>
#include <memory>
#include <iostream>
#include <thread>
#include <functional>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <QApplication>
#include <QLabel>
#include <QImage>

class MjpegView
{
public:
  MjpegView(std::string target, std::string topic)
  {
    std::string uri = "http://" + target + "/stream?topic=" + topic;
    if (!vcap_.open(uri))
    {
      std::cout << "Error opening video stream or file" << std::endl;
      return;
    }
    label_ = new QLabel("######## basic1 ########");
    label_->show();
    recv_thread_ = std::thread(std::bind(&MjpegView::loop, this));
  }

private:
  void loop(void)
  {
    while (true)
    {
      cv::Mat image;
      if (!vcap_.read(image))
      {
        std::cout << "No frame" << std::endl;
        return;
      }
      cv::Mat rgb_image;
      cv::cvtColor(image, rgb_image, cv::COLOR_BGR2RGB);
      QImage qimage(rgb_image.data, rgb_image.cols, rgb_image.rows, QImage::QImage::Format_RGB888);
      label_->resize(rgb_image.cols, rgb_image.rows);
      label_->setPixmap(QPixmap::fromImage(qimage));
    }
  }

  cv::VideoCapture vcap_;
  QLabel* label_;
  std::thread recv_thread_;
  std::thread qt_thread_;
};

int main(int argc, char** argv)
{
  QApplication app(argc, argv);
  MjpegView mjpeg_view("localhost:8080", "/camera/image_raw");
  return app.exec();
}