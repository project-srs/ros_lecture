#include "mbed.h"
#include "rtos.h"

#define SRSG_LED0 P0_16
#define SRSG_LED1 P0_15
#define SRSG_LED2 P0_17
#define SRSG_LED3 P0_18

DigitalOut l0(SRSG_LED0);
DigitalOut l1(SRSG_LED1);
DigitalOut l2(SRSG_LED2);
DigitalOut l3(SRSG_LED3);

void blink1(void) {
    while (1) {
        l1=!l1;
        Thread::wait(1000);
    }
}
void blink2(void) {
    while (1) {
        l2=!l2;
        Thread::wait(1000);
    }
}

int main() {
    Thread thread1;
    thread1.start(blink1);
    Thread thread2;
    thread2.start(blink2);
    while (true) {
        l0 = !l0;
        Thread::wait(500);
    }
}
