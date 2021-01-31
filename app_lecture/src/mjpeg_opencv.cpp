#include <unistd.h>
#include <memory>
#include <iostream>
#include <thread>
#include <functional>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

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
    cv::namedWindow("Output Window");
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
      cv::imshow("Output Window", image);
      cv::waitKey(1);
    }
  }

  cv::VideoCapture vcap_;
  std::thread recv_thread_;
};

int main(int, char**)
{
  MjpegView mjpeg_view("localhost:8080", "/camera/image_raw");
  while (true)
  {
    sleep(1);
  }
}