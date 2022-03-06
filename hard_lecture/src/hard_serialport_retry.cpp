#include "ros/ros.h"
#include "std_msgs/String.h"
#include "diagnostic_updater/diagnostic_updater.h"

#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

class serial_stream{
private:
	int fd;
	std::string device_name;
	bool connected;
	int read_success;
public:
	serial_stream(void){
		connected=false;
		read_success=0;
	}
	void set_name(std::string name){
		device_name=name;
		
	}
	void ss_open(void){
		fd=open(device_name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
		fcntl(fd, F_SETFL,0);
		//load configuration
		struct termios conf_tio;
		tcgetattr(fd,&conf_tio);
		//set baudrate
		speed_t BAUDRATE = B1000000;
		cfsetispeed(&conf_tio, BAUDRATE);
		cfsetospeed(&conf_tio, BAUDRATE);
		//non canonical, non echo back
		conf_tio.c_lflag &= ~(ECHO | ICANON);
		//non blocking
		conf_tio.c_cc[VMIN]=0;
		conf_tio.c_cc[VTIME]=0;
		//store configuration
		tcsetattr(fd,TCSANOW,&conf_tio);
		if(fd>=0){
			connected=true;
		}
		else{
			connected=false;
		}
	}
	void ss_write(std::string data0){
		if(connected){
			int rec=write(fd,data0.c_str(),data0.size());
			if(rec<0){
				connected=false;
				ss_close();
			}
		}
	}
	std::string ss_read(void){
		if(connected){
			char buf[256]={0};
			int recv_data=read(fd, buf, sizeof(buf));
			if(recv_data>0){
				read_success++;
				std::string recv_string=buf;
				return buf;
			}
			else{
				return "";
			}
		}
		return "";
	}
	void ss_close(void){
		close(fd);
		connected=false;
	}
	bool ss_connected(void){
		return connected;
	}
	bool ss_status(void){
		if(read_success>0){
			read_success=0;
			return true;
		}
		else return false;
	}
};
diagnostic_updater::Updater *p_updater;
ros::Publisher serial_pub;
std::string device_name="/dev/ttyUSB0";
serial_stream ss0;

void serial_callback(const std_msgs::String& serial_msg){
	ss0.ss_write(serial_msg.data);
}
bool first_time=true;
bool last_connected=false;
void timer_callback(const ros::TimerEvent&){
		if(!ss0.ss_connected()){
			ss0.ss_open();
			if(ss0.ss_connected())ROS_INFO("Serial Open %s", device_name.c_str());
			else{
				if     (first_time)    ROS_ERROR("Serial Fail: cound not open %s", device_name.c_str());
				else if(last_connected)ROS_ERROR("Serial Fail: Connection is broken %s", device_name.c_str());
			}
		}
		else{
			std::string recv_data=ss0.ss_read();
			if(recv_data.size()>0){
				//printf("recv:%03d %s\n",(int)recv_data.size(),recv_data.c_str());
				std_msgs::String serial_msg;
				serial_msg.data=recv_data;
				serial_pub.publish(serial_msg);
			}
		}
		first_time=false;
		last_connected=ss0.ss_connected();
		p_updater->update();		
		ros::spinOnce();
}

void diagnostic0(diagnostic_updater::DiagnosticStatusWrapper &stat){
	bool serial_c=ss0.ss_connected();
	bool serial_s=ss0.ss_status();
	if     (serial_c &&  serial_s)stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK,    "Active.");
	else if(serial_c && !serial_s)stat.summaryf(diagnostic_msgs::DiagnosticStatus::WARN,  "No Recieve.");
	else		                  stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "No Connection.");
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "hard_serialport_retry");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	//param
	pn.getParam("device_name", device_name);
	ss0.set_name(device_name);
	//Publisher
	serial_pub = n.advertise<std_msgs::String>("Serial_in", 10);
	//timer
	ros::Timer timer = n.createTimer(ros::Duration(0.01), timer_callback);
	//Diagnostic
	diagnostic_updater::Updater updater;
	p_updater=&updater;
	updater.setHardwareID("SerialPort");
	updater.add("Connect", diagnostic0);
	//Subscriber
	ros::Subscriber serial_sub = n.subscribe("Serial_out", 10, serial_callback); 
			
	ros::spin();
	ss0.ss_close();
 	return 0;
}

