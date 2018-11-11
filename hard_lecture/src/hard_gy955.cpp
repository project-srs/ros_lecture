#include "ros/ros.h"
#include "sensor_msgs/Imu.h"

#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

int open_serial(const char *device_name){
	int fd1=open(device_name, O_RDWR | O_NOCTTY | O_NONBLOCK);
	fcntl(fd1, F_SETFL,0);
	//load configuration
	struct termios conf_tio;
	tcgetattr(fd1,&conf_tio);
	//set baudrate
	speed_t BAUDRATE = B9600;
	cfsetispeed(&conf_tio, BAUDRATE);
	cfsetospeed(&conf_tio, BAUDRATE);
	//non canonical, non echo back
	conf_tio.c_lflag &= ~(ECHO | ICANON);
	//non blocking
	conf_tio.c_cc[VMIN]=0;
	conf_tio.c_cc[VTIME]=0;
	//store configuration
	tcsetattr(fd1,TCSANOW,&conf_tio);
	return fd1;
}

int fd1;
int main(int argc, char **argv)
{
	ros::init(argc, argv, "s4_comport_serialport");
	ros::NodeHandle n;

	//Publisher
	ros::Publisher imu_pub  = n.advertise<sensor_msgs::Imu >("imu", 10);
	
	char device_name[]="/dev/ttyUSB0";
	fd1=open_serial(device_name);
	if(fd1<0){
		ROS_ERROR("Serial Fail: cound not open %s", device_name);
		printf("Serial Fail\n");
		ros::shutdown();
	}

	char send_data[3]={170,16,10};
	int rec=write(fd1,send_data,3);
	if(rec==0){
		ROS_ERROR_ONCE("Serial Fail: cound not write");
	}

	ros::Rate loop_rate(100); 
	static unsigned char buffer_data[256]={0};
	static int buffer_size=0;
	while (ros::ok()){
		unsigned char recv_data[256]={0};
		int recv_size=read(fd1, recv_data, sizeof(recv_data));
		if(recv_size>0){
			//add
			for(int i=0;i<recv_size;i++){
				buffer_data[buffer_size+i]=recv_data[i];
			}
			buffer_size+=recv_size;

			//dencode
			int state=0;//0:serch 1st 90, 1:serch 2nd 90, 2:check size, 3:end
			int next_index=0;
			for(int i=0;i<buffer_size;i++){
				if(state==0 || state==1){
					if(buffer_data[i]==90)state++;	
				}
				else if(state==2){
					if(i+11<buffer_size){
						int mode=buffer_data[i+0];
						int size=buffer_data[i+1];
						int16_t tmp_w=(buffer_data[i+2]*256+buffer_data[i+3]);
						float quat_w=tmp_w/10000.0;
						int16_t tmp_x=(buffer_data[i+4]*256+buffer_data[i+5]);
						float quat_x=tmp_x/10000.0;
						int16_t tmp_y=(buffer_data[i+6]*256+buffer_data[i+7]);
						float quat_y=tmp_y/10000.0;
						int16_t tmp_z=(buffer_data[i+8]*256+buffer_data[i+9]);
						float quat_z=tmp_z/10000.0;
						state++;
						next_index=i+12;
						float quat_size=quat_w * quat_w + quat_x * quat_x + quat_y * quat_y + quat_z * quat_z;
						if(0.9<quat_size && quat_size<1.1){
							//publish
							sensor_msgs::Imu imu_msg;
							imu_msg.header.frame_id="map";
							imu_msg.header.stamp=ros::Time::now();
							imu_msg.orientation.x=quat_x;
							imu_msg.orientation.y=quat_y;
							imu_msg.orientation.z=quat_z;
							imu_msg.orientation.w=quat_w;
							imu_pub.publish(imu_msg);
						}
						else{
							ROS_WARN("quat size error: %f",quat_size);
						}
					}
				}
			}
			//sub
			if(next_index>0){
				for(int i=0;i<buffer_size-next_index;i++){
					buffer_data[i]=buffer_data[i+next_index];
				}
				buffer_size-=next_index;
			}
		}
		ros::spinOnce();
		loop_rate.sleep();
	} 
 	return 0;
}
