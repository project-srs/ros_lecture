#include "ros/ros.h"
#include "sensor_msgs/Imu.h"

#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <deque>

class GY955Driver
{
public:
	ros::NodeHandle nh_;
	ros::NodeHandle pnh_;
	ros::Publisher imu_pub_;
	int fd1_;
	std::deque<unsigned char> buffer_data_;
	std::string device_name_;

	GY955Driver() : nh_(), pnh_("~")
	{
		//Publisher
		imu_pub_ = nh_.advertise<sensor_msgs::Imu>("imu", 10);

		device_name_= "/dev/ttyUSB0";
		pnh_.getParam("device_name", device_name_);

		fd1_ = open_serial(device_name_);
		if (fd1_ < 0)
		{
			ROS_ERROR("Serial Fail: cound not open %s", device_name_.c_str());
			printf("Serial Fail\n");
			ros::shutdown();
		}

		std::vector<unsigned  char> send_data({170, 16, 10});
		int rec=write_serial(fd1_, send_data);
		//unsigned char send_data[3] = {170, 16, 10};
		//int rec = write(fd1_, send_data, 3);
		if (rec == 0)
		{
			ROS_ERROR("Serial Fail: cound not write");
		}

		ros::Rate loop_rate(100);
		while (ros::ok())
		{
			unsigned char recv_data[256] = {0};
			int recv_size = read(fd1_, recv_data, sizeof(recv_data));
			if (recv_size > 0)
			{
				//add
				for (int i = 0; i < recv_size; i++)
				{
					buffer_data_.push_back(recv_data[i]);
				}

				for (int i = 0; i < (int)buffer_data_.size() - 14; i++)
				{
					std::vector<int> data;
					if (buffer_data_[i] == 90 && buffer_data_[i + 1] == 90 && buffer_data_[i + 2] == 16 && buffer_data_[i + 3] == 9)
					{
						for (int j = i; j < i + 14; j++)
						{
							data.push_back(buffer_data_[j]);
							//printf("[%i]:%i\n",j,buffer_data_[j]);
						}
						//printf("\n");
						for (int j = 0; j < i + 14; j++)
						{
							buffer_data_.pop_front();
						}

						print_data(data);
						break;
					}
				}
			}
			ros::spinOnce();
			loop_rate.sleep();
		}
	}

	int open_serial(const std::string device_name)
	{
		int fd1 = open(device_name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
		fcntl(fd1, F_SETFL, 0);
		//load configuration
		struct termios conf_tio;
		tcgetattr(fd1, &conf_tio);
		//set baudrate
		speed_t BAUDRATE = B9600;
		cfsetispeed(&conf_tio, BAUDRATE);
		cfsetospeed(&conf_tio, BAUDRATE);
		//non canonical, non echo back
		conf_tio.c_lflag &= ~(ECHO | ICANON);
		//non blocking
		conf_tio.c_cc[VMIN] = 0;
		conf_tio.c_cc[VTIME] = 0;
		//store configuration
		tcsetattr(fd1, TCSANOW, &conf_tio);
		return fd1;
	}

	int write_serial(int fd1, std::vector<unsigned char> data){
		return write(fd1, data.data(), (int)data.size());
	}


	int print_data(std::vector<int> data)
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
			//publish
			sensor_msgs::Imu imu_msg;
			imu_msg.header.frame_id = "map";
			imu_msg.header.stamp = ros::Time::now();
			imu_msg.orientation.x = quat_x;
			imu_msg.orientation.y = quat_y;
			imu_msg.orientation.z = quat_z;
			imu_msg.orientation.w = quat_w;
			imu_pub_.publish(imu_msg);
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
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "gy955_driver");
	GY955Driver gy955_driver;
	ros::spin();
}
