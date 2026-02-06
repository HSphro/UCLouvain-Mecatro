#include <stdio.h>
#include <pigpio.h>
#include <stdint.h>
#include <unistd.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <signal.h>
#include <stdbool.h>
#include <math.h>

// --- Hardware Pins (BCM) ---
#define LEFT_PWM  12
#define LEFT_DIR  5
#define RIGHT_PWM 13
#define RIGHT_DIR 6

// --- Constraints ---
#define CLAMP_MAX 255.0
#define CLAMP_MIN 0.0

// --- SPI & Physics ---
#define ADDR_ODO_A 0x10
#define ADDR_ODO_B 0x20
#define ODO_PPR 2048.0
#define WHEEL_RADIUS_M 0.019 
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

volatile bool running = true;
int fd_spi;

typedef struct {
    double target;
    double integral;
    double prev_error;
    double Kp;
    double Ki;
    double Kd;
} PIDState;

void handle_sigint(int sig) { running = false; }

int32_t read_fpga(uint8_t addr) {
    uint8_t tx1[] = {addr, 0, 0, 0}, rx1[4];
    struct spi_ioc_transfer tr1 = { .tx_buf = (unsigned long)tx1, .rx_buf = (unsigned long)rx1, .len = 4, .speed_hz = 500000 };
    ioctl(fd_spi, SPI_IOC_MESSAGE(1), &tr1);
    
    uint8_t tx2[] = {0, 0, 0, 0}, rx2[4];
    struct spi_ioc_transfer tr2 = { .tx_buf = (unsigned long)tx2, .rx_buf = (unsigned long)rx2, .len = 4, .speed_hz = 500000 };
    ioctl(fd_spi, SPI_IOC_MESSAGE(1), &tr2);
    return (rx2[0] << 24) | (rx2[1] << 16) | (rx2[2] << 8) | rx2[3];
}

int calculate_pid(PIDState *pid, double current_val, double dt) {
    double error = pid->target - current_val;
    pid->integral += error * dt;
    double D = pid->Kd * (error - pid->prev_error) / dt;
    double output = (pid->Kp * error) + (pid->Ki * pid->integral) + D;
    pid->prev_error = error;

    if (output > CLAMP_MAX) {
        output = CLAMP_MAX;
        pid->integral -= error * dt; 
    } else if (output < CLAMP_MIN) {
        output = CLAMP_MIN;
        pid->integral -= error * dt;
    }
    return (int)output;
}

int main() {
    // 1. Initialize pigpio
    if (gpioInitialise() < 0) {
        printf("pigpio initialization failed!\n");
        return 1;
    }

    // 2. SPI Auto-detection
    const char *device = "/dev/spidev0.0";
    if (access("/dev/spidev0.1", F_OK) == 0) device = "/dev/spidev0.1";
    
    fd_spi = open(device, O_RDWR);
    if (fd_spi < 0) {
        perror("SPI Open Failed (Did you enable SPI in raspi-config?)");
        gpioTerminate();
        return 1;
    }

    signal(SIGINT, handle_sigint);

    // 3. Setup Motor Pins (Matching your working PWM script)
    gpioSetMode(LEFT_DIR, PI_OUTPUT);
    gpioSetMode(RIGHT_DIR, PI_OUTPUT);
    gpioWrite(LEFT_DIR, 1);  // Forward
    gpioWrite(RIGHT_DIR, 1); // Forward

    gpioSetPWMrange(LEFT_PWM, 255);
    gpioSetPWMrange(RIGHT_PWM, 255);

    // 4. PID Initialization
    PIDState pidL = {0.6, 0, 0, 290.0, 20.0, 1.0}; 
    PIDState pidR = {0.6, 0, 0, 300.0, 20.0, 0.0}; 

    const double m_per_tick = (2 * M_PI * WHEEL_RADIUS_M) / ODO_PPR;
    int32_t last_odo_a = -read_fpga(ADDR_ODO_A);
    int32_t last_odo_b = read_fpga(ADDR_ODO_B);
    
    double dt = 0.02; 
    int zero_move_count = 0;
    int pwmL = 0, pwmR = 0;

    printf("PID Control Active. Device: %s | Target: %.2f m/s\n", device, pidL.target);

    while (running) {
        int32_t odo_a = -read_fpga(ADDR_ODO_A);
        int32_t odo_b = read_fpga(ADDR_ODO_B);

        // Fail-safe
        if ((pwmL > 60 || pwmR > 60) && (odo_a == last_odo_a && odo_b == last_odo_b)) {
            zero_move_count++;
        } else { zero_move_count = 0; }

        if (zero_move_count > 25) {
            printf("\nEMERGENCY STOP: Encoder failure!\n");
            break;
        }

        double speedL = ((odo_a - last_odo_a) * m_per_tick) / dt;
        double speedR = ((odo_b - last_odo_b) * m_per_tick) / dt;

        pwmL = calculate_pid(&pidL, speedL, dt);
        pwmR = calculate_pid(&pidR, speedR, dt);

        gpioPWM(LEFT_PWM, pwmL);
        gpioPWM(RIGHT_PWM, pwmR);

        printf("\rL: %.2f m/s (PWM: %3d) | R: %.2f m/s (PWM: %3d)", speedL, pwmL, speedR, pwmR);
        fflush(stdout);

        last_odo_a = odo_a; last_odo_b = odo_b;
        gpioDelay(20000); 
    }

    // Stop Motors
    gpioPWM(LEFT_PWM, 0);
    gpioPWM(RIGHT_PWM, 0);
    gpioTerminate();
    close(fd_spi);
    printf("\nClean shutdown complete.\n");
    return 0;
}

//gcc -o PID PID.c -lpigpio -lrt -lpthread -lm