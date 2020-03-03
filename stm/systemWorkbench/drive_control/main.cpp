#include "motor.h"
#include "stm_bldc_motor.h"

using namespace KATBOT;

void main()
{
  Motor *stmBldcMotor = new StmBLDCMotor();
  stmBldcMotor->init();
}


/*#include "mbed.h"
#include "STSpin240_250.h"

#include <std_msgs/String.h>

void print_char(char c = '*')
{
    printf("%c", c);
    fflush(stdout);
}

Thread thread;

DigitalOut led1(LED1);

void print_thread()
{
    while (true) {
        wait(1);
        print_char();
    }
}

int main()
{
    printf("\n\n*** RTOS basic example ***\n");

    thread.start(print_thread);

    while (true) {
        led1 = !led1;
        wait(0.5);
    }
}*/
