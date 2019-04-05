#include <mbed.h>
#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>

DigitalOut led = P0_16;

ros::NodeHandle  nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

void messageCb( const std_msgs::Empty& toggle_msg) {
  led=1;
  wait(0.2);
  led=0;
}
ros::Subscriber<std_msgs::Empty> sub("led", &messageCb );

int main() {

  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub);

  while (1) {
    str_msg.data = "hello world!";
    chatter.publish( &str_msg );
    nh.spinOnce();
    wait_ms(1000);
  }
}