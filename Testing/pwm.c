#include <stdio.h>
#include <pigpio.h>
#include <signal.h>
#include <stdbool.h>

#define LEFT_PWM  12  // BCM pin numbers
#define LEFT_DIR  5
#define RIGHT_PWM 13
#define RIGHT_DIR 6

volatile bool running = true;

void handle_sigint(int sig) {
    running = false;
}

int main() {
    if (gpioInitialise() < 0) {
        printf("pigpio initialization failed!\n");
        return 1;
    }

    signal(SIGINT, handle_sigint);

    // Set direction pins
    gpioSetMode(LEFT_DIR, PI_OUTPUT);
    gpioSetMode(RIGHT_DIR, PI_OUTPUT);
    gpioWrite(LEFT_DIR, 1);   // FORWARD
    gpioWrite(RIGHT_DIR, 1);  // FORWARD

    // Set PWM range and duty cycle
    gpioSetPWMrange(LEFT_PWM, 255);
    gpioSetPWMrange(RIGHT_PWM, 255);
    gpioPWM(LEFT_PWM, 70);    // 20% of 255
    gpioPWM(RIGHT_PWM, 70);   // 20% of 255

    printf("Motors running at 20%% duty cycle\n");

    int elapsed = 0;
    while (running && elapsed < 2000) {  // 2000 ms = 2 seconds
        gpioDelay(1000); // 1 ms delay
        elapsed++;
    }

    // Stop motors
    gpioPWM(LEFT_PWM, 0);
    gpioPWM(RIGHT_PWM, 0);
    printf("Motors stopped\n");

    gpioTerminate();
    return 0;
}


//gcc pwm.c -lpigpio -lrt -lpthread -o pwm
//sudo systemctl stop or stop pigpiod
//sudo ./pwm
