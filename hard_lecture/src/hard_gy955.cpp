#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "diagnostic_updater/diagnostic_updater.h"

#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <deque>
#include <sys/ioctl.h>

class serial_stream
{
private:
  int fd_;
  std::string device_name_;
  int baud_rate_;
  bool connected_;
  int read_bytes_;

public:
  //0: OK, 1: Not found, 2: Can not Write, 3: Can not Read 4: Connection broken
  int close_reason_;

public:
  serial_stream(void)
  {
    connected_ = false;
    read_bytes_ = 0;
    close_reason_ = 0;
  }

  void ssOpen(std::string device_name, int baud_rate)
  {
    device_name_ = device_name;
    baud_rate_ = baud_rate;
    openSerial_();
  }

  bool ssReOpen(void)
  {
    if (!connected_)
    {
      openSerial_();
      return connected_;
    }
    else
    {
      return false;
    }
  }

  void openSerial_(void)
  {
    fd_ = open(device_name_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    fcntl(fd_, F_SETFL, 0);
    //load configuration
    struct termios conf_tio;
    tcgetattr(fd_, &conf_tio);
    //set baudrate
    speed_t BAUDRATE;
    if (baud_rate_ == 9600)
    {
      BAUDRATE = B9600;
    }
    else if (baud_rate_ == 1000000)
    {
      BAUDRATE = B1000000;
    }
    else
    {
      connected_ = false;
      close_reason_ = 1;
      return;
    }
    cfsetispeed(&conf_tio, BAUDRATE);
    cfsetospeed(&conf_tio, BAUDRATE);
    //non canonical, non echo back
    conf_tio.c_lflag &= CS8;
    //conf_tio.c_lflag &= ~(ECHO | ICANON);
    //non blocking
    conf_tio.c_cc[VMIN] = 0;
    conf_tio.c_cc[VTIME] = 0;
    //store configuration
    tcsetattr(fd_, TCSANOW, &conf_tio);
    if (fd_ >= 0)
    {
      connected_ = true;
      close_reason_ = 0;
    }
    else
    {
      connected_ = false;
      close_reason_ = 1;
    }
  }

  bool ssCheckStatus(void)
  {
    if (connected_)
    {
      int stat;
      int res;
      res= ioctl(fd_, TIOCMGET, &stat);
      if(stat & TIOCM_CD)
      {
        ssClose();
        connected_ = false;
        close_reason_ = 4;
        return true;      
      }
    }
    return false;
  }

  void ssClose(void)
  {
    close(fd_);
    connected_ = false;
    close_reason_ = 0;
  }

  void ssWrite(std::vector<unsigned char> send_data)
  {
    if (connected_ && (int)send_data.size() > 0)
    {
      int rec = write(fd_, send_data.data(), (int)send_data.size());
      if (rec <= 0)
      {
        connected_ = false;
        close_reason_ = 2;
        ssClose();
      }
    }
  }

  std::vector<unsigned char> ssRead(void)
  {
    if (connected_)
    {
      char buf[256] = {0};
      int recv_size = read(fd_, buf, sizeof(buf));
      if (recv_size > 0)
      {
        read_bytes_ += recv_size;
        std::vector<unsigned char> recv_data;
        recv_data.resize(recv_size);
        for (int i = 0; i < recv_size; i++)
          recv_data[i] = buf[i];
        return recv_data;
      }
    }
    std::vector<unsigned char> null_data;
    return null_data;
  }

  bool isConnected(void)
  {
    return connected_;
  }

  int getReadBytes(void){
    int tmp = read_bytes_;
    read_bytes_ = 0;
    return tmp;
  }
};

void print_vecter(std::vector<unsigned char> data)
{
  printf("size: %i\n", (int)data.size());
  for (int i = 0; i < (int)data.size(); i++)
  {
    printf("%03i,", data[i]);
  }
  printf("\n");
}

void print_vecter(std::deque<unsigned char> data)
{
  printf("size: %i\n", (int)data.size());
  for (int i = 0; i < (int)data.size(); i++)
  {
    printf("%03i,", data[i]);
  }
  printf("\n");
}

class GY955Driver
{
public:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher imu_pub_;
  diagnostic_updater::Updater updater_;
  ros::Timer timer1_;
  ros::Timer timer2_;
  serial_stream ss0_;
  std::deque<unsigned char> buffer_data_;
  std::string device_name_;
  std::string imu_frame_name_;
  bool debug_;
  GY955Driver() : nh_(), pnh_("~")
  {
    //Publisher
    imu_pub_ = nh_.advertise<sensor_msgs::Imu>("imu", 10);
    //Param
    device_name_ = "/dev/ttyUSB0";
    imu_frame_name_ = "imu_link";
    pnh_.getParam("imu_frame_name", imu_frame_name_);
    debug_ = false;
    pnh_.getParam("debug", debug_); 
    //Diagnostic
  	updater_.setHardwareID("SerialPort");
	  updater_.add("Connect",  boost::bind(&GY955Driver::diag_callback, this, _1));

    ss0_.ssOpen(device_name_, 9600);

    if (!ss0_.isConnected())
    {
      ROS_ERROR("Serial Fail: cound not open %s", device_name_.c_str());
      ros::shutdown();
    }

    std::vector<unsigned char> send_data({170, 16, 10});
    ss0_.ssWrite(send_data);
    if (!ss0_.isConnected())
    {
      ROS_ERROR("Serial Fail: cound not write %s", device_name_.c_str());
      ros::shutdown();
    }
    timer1_ = nh_.createTimer(ros::Duration(0.02), &GY955Driver::timer1_callback, this);
    timer2_ = nh_.createTimer(ros::Duration(1.00), &GY955Driver::timer2_callback, this);
  }

  void timer1_callback(const ros::TimerEvent &)
  {
    std::vector<unsigned char> recv_data = ss0_.ssRead();
    if ((int)recv_data.size() > 0)
    {
      //add
      for (int i = 0; i < (int)recv_data.size(); i++)
      {
        buffer_data_.push_back(recv_data[i]);
      }

      //extract
      for (int i = 0; i < (int)buffer_data_.size() - 14; i++)
      {
        std::vector<unsigned char> data;
        if (buffer_data_[i] == 90 && buffer_data_[i + 1] == 90 && buffer_data_[i + 2] == 16 && buffer_data_[i + 3] == 9)
        {
          for (int j = i; j < i + 14; j++)
          {
            data.push_back(buffer_data_[j]);
          }

          for (int j = 0; j < i + 14; j++)
          {
            buffer_data_.pop_front();
          }

          if (checkSum(data))
          {
            sensor_msgs::Imu imu_data = convert_data(data);
            if (checkSize(imu_data))
              imu_pub_.publish(imu_data);
            else
              if(debug_)ROS_ERROR("NG size");
          }
          else
            if(debug_)ROS_ERROR("NG checksum");
            //print_data(data);
          break;
        }
      }
    }
  }

  void timer2_callback(const ros::TimerEvent &)
  {
    if (ss0_.ssCheckStatus())
    {
      ROS_ERROR("Connection Broken %s", device_name_.c_str());
    }
    if (ss0_.ssReOpen())
    {
      ROS_INFO("ReOpen %s", device_name_.c_str());
      
      std::vector<unsigned char> send_data({170, 16, 10});
      ss0_.ssWrite(send_data);
      if (!ss0_.isConnected())
      {
        ROS_ERROR("Serial Fail: cound not write %s", device_name_.c_str());
        ros::shutdown();
      }
    }
    updater_.update();
  }

  bool checkSum(std::vector<unsigned char> data)
  {
    if ((int)data.size() >= 2)
    {
      int sum = 0;
      for (int i = 0; i < (int)data.size() - 1; i++)
      {
        sum += data[i];
      }
      if (sum % 256 == data.back() % 256)
        return true;
      else
        return false;
    }
    return false;
  }
  void diag_callback(diagnostic_updater::DiagnosticStatusWrapper &stat){
    bool serial_c = ss0_.isConnected();
    bool serial_s = ss0_.getReadBytes() > 0;
    if     (serial_c &&  serial_s)stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK,    "Active.");
    else if(serial_c && !serial_s)stat.summaryf(diagnostic_msgs::DiagnosticStatus::WARN,  "No Recieve.");
    else		                  stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "No Connection.");
  }
  int print_data(std::vector<unsigned char> data)
  {
    int mode = data[2];
    int size = data[3];
    int16_t tmp_w = (data[4] * 256 + data[5]);
    float quat_w = tmp_w / 10000.0;
    int16_t tmp_x = (data[6] * 256 + data[7]);
    float quat_x = tmp_x / 10000.0;
    int16_t tmp_y = (data[8] * 256 + data[9]);
    float quat_y = tmp_y / 10000.0;
    int16_t tmp_z = (data[10] * 256 + data[11]);
    float quat_z = tmp_z / 10000.0;
    int d0 = (data[12] >> 6) & 0x03;
    int d1 = (data[12] >> 4) & 0x03;
    int d2 = (data[12] >> 2) & 0x03;
    int d3 = (data[12] >> 0) & 0x03;
    float quat_size = quat_w * quat_w + quat_x * quat_x + quat_y * quat_y + quat_z * quat_z;
    if (0.9 < quat_size && quat_size < 1.1)
    {
      printf("OK\n");
    }
    else
    {
      printf("NG\n");
    }
    printf("mag:%f\n", quat_size);
    int sum = 0;
    for (int j = 0; j < (int)data.size(); j++)
    {
      printf("%03i,", data[j]);
      sum += data[j];
    }
    printf(":%i %i\n", (int)data.size(), sum % 256);
  }

  sensor_msgs::Imu convert_data(std::vector<unsigned char> data)
  {
    int mode = data[2];
    int size = data[3];
    int16_t tmp_w = (data[4] * 256 + data[5]);
    float quat_w = tmp_w / 10000.0;
    int16_t tmp_x = (data[6] * 256 + data[7]);
    float quat_x = tmp_x / 10000.0;
    int16_t tmp_y = (data[8] * 256 + data[9]);
    float quat_y = tmp_y / 10000.0;
    int16_t tmp_z = (data[10] * 256 + data[11]);
    float quat_z = tmp_z / 10000.0;
    int d0 = (data[12] >> 6) & 0x03;
    int d1 = (data[12] >> 4) & 0x03;
    int d2 = (data[12] >> 2) & 0x03;
    int d3 = (data[12] >> 0) & 0x03;
    float quat_size = quat_w * quat_w + quat_x * quat_x + quat_y * quat_y + quat_z * quat_z;

    sensor_msgs::Imu imu_msg;
    imu_msg.header.frame_id = "map";
    imu_msg.header.stamp = ros::Time::now();
    imu_msg.orientation.x = quat_x;
    imu_msg.orientation.y = quat_y;
    imu_msg.orientation.z = quat_z;
    imu_msg.orientation.w = quat_w;

    return imu_msg;
  }

  bool checkSize(sensor_msgs::Imu imu_msg)
  {
    float x2 = imu_msg.orientation.x * imu_msg.orientation.x;
    float y2 = imu_msg.orientation.y * imu_msg.orientation.y;
    float z2 = imu_msg.orientation.z * imu_msg.orientation.z;
    float w2 = imu_msg.orientation.w * imu_msg.orientation.w;
    float size2 = x2 + y2 + z2 + w2;
    if (0.9 < size2 && size2 < 1.1)
      return true;
    return false;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gy955_driver");
  ros::NodeHandle nh;
  GY955Driver gy955_driver;
  ros::spin();
}
