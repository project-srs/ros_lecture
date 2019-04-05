#include "mbed.h"
#define SRSG_LED0 P0_16
#define SRSG_TX0 P0_2
#define SRSG_RX0 P0_3

DigitalOut l0(SRSG_LED0);
Serial pc(SRSG_TX0,SRSG_RX0);

int main() {
  pc.baud(9600);
  pc.printf("start\n");
  while (true) {
    l0 = 0;
    wait(1);
    l0 = 1;
    wait(1);
    pc.printf("hellow mbed!\n");
  }
  return 0;
}