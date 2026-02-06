#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <time.h>
#include <math.h>

// ==========================================
// CONFIGURATION
// ==========================================
#define SPI_SPEED 500000 

// FPGA Addresses
#define ADDR_ODO_A 0x10
#define ADDR_ODO_B 0x20
#define ADDR_ENC_A 0x13
#define ADDR_ENC_B 0x23

// Physical Constants
#define ODO_PPR 2048.0
#define WHEEL_RADIUS_M 0.019 //0.043/2 
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

int fd_spi;

// ==========================================
// SPI SETUP
// ==========================================
void spi_init() {
    const char *device = "/dev/spidev0.0";
    if (access("/dev/spidev0.1", F_OK) == 0) device = "/dev/spidev0.1";
    
    fd_spi = open(device, O_RDWR);
    if (fd_spi < 0) {
        perror("Error opening SPI");
        exit(1);
    }

    uint8_t mode = 0;
    uint8_t bits = 8;
    uint32_t speed = SPI_SPEED;
    ioctl(fd_spi, SPI_IOC_WR_MODE, &mode);
    ioctl(fd_spi, SPI_IOC_WR_BITS_PER_WORD, &bits);
    ioctl(fd_spi, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
}

// ==========================================
// READ FUNCTION
// ==========================================
int32_t read_fpga_register(uint8_t addr) {
    uint8_t tx1[] = {addr, 0, 0, 0};
    uint8_t rx1[] = {0, 0, 0, 0}; 
    struct spi_ioc_transfer tr1 = { .tx_buf = (unsigned long)tx1, .rx_buf = (unsigned long)rx1, .len = 4, .speed_hz = SPI_SPEED, .bits_per_word = 8 };
    ioctl(fd_spi, SPI_IOC_MESSAGE(1), &tr1);

    uint8_t tx2[] = {0, 0, 0, 0};
    uint8_t rx2[] = {0, 0, 0, 0};
    struct spi_ioc_transfer tr2 = { .tx_buf = (unsigned long)tx2, .rx_buf = (unsigned long)rx2, .len = 4, .speed_hz = SPI_SPEED, .bits_per_word = 8 };
    ioctl(fd_spi, SPI_IOC_MESSAGE(1), &tr2);

    return (rx2[0] << 24) | (rx2[1] << 16) | (rx2[2] << 8) | rx2[3];
}

double get_time_s() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec / 1e9;
}

// ==========================================
// MAIN LOOP
// ==========================================
int main() {
    spi_init();

    // Clear screen once to start clean
    printf("\033[2J\033[H"); 
    printf("=== FPGA Sensor Dashboard ===\n\n");

    // History variables
    int32_t prev_odo_a = 0, prev_odo_b = 0;
    int32_t start_odo_a = 0, start_odo_b = 0;
    double prev_time = 0.0;
    int first_run = 1;

    // Physics constant
    const double meters_per_tick = (2 * M_PI * WHEEL_RADIUS_M) / ODO_PPR;

    while(1) {
        // 1. Read
        int32_t odo_a = read_fpga_register(ADDR_ODO_A);
        int32_t odo_b = read_fpga_register(ADDR_ODO_B);

        // 2. Invert A
        odo_a = -odo_a;

        // 3. Time
        double current_time = get_time_s();
        double speed_a = 0.0, speed_b = 0.0;
        double dist_a = 0.0, dist_b = 0.0;

        // 4. Calculations
        if (first_run) {
            start_odo_a = odo_a;
            start_odo_b = odo_b;
            prev_odo_a = odo_a;
            prev_odo_b = odo_b;
            first_run = 0;
        } else {
            double dt = current_time - prev_time;
            if (dt > 0.0001) {
                speed_a = ((odo_a - prev_odo_a) * meters_per_tick) / dt;
                speed_b = ((odo_b - prev_odo_b) * meters_per_tick) / dt;
            }
        }

        // Distance from start
        dist_a = (odo_a - start_odo_a) * meters_per_tick;
        dist_b = (odo_b - start_odo_b) * meters_per_tick;

        prev_odo_a = odo_a;
        prev_odo_b = odo_b;
        prev_time = current_time;

        // 5. Multi-line Print
        // \033[K clears the rest of the line to prevent artifacts when numbers shrink
        printf("TICKS:    A: %-10d | B: %-10d   \033[K\n", odo_a, odo_b);
        printf("SPEED:    A: %-6.2f m/s  | B: %-6.2f m/s  \033[K\n", speed_a, speed_b);
        printf("DISTANCE: A: %-6.2f m    | B: %-6.2f m    \033[K\n", dist_a, dist_b);
        
        // Move cursor UP 3 lines so we overwrite next time
        printf("\033[3A");

        fflush(stdout);
        usleep(50000); // 50ms
    }

    close(fd_spi);
    return 0;
}