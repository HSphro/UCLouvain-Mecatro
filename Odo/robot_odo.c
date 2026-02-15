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
#include <arpa/inet.h>
#include <sys/socket.h>
#include <inttypes.h>

// ==========================================
// CONFIGURATION
// ==========================================
#define SPI_SPEED 500000 
#define ADDR_ODO_A 0x10
#define ADDR_ODO_B 0x20

// --- ROBOT PHYSICAL CONSTANTS (YOU MUST MEASURE THESE) ---
#define ODO_PPR 2048.0
#define WHEEL_RADIUS_M 0.019
#define WHEEL_BASE_M 0.24  // <--- DISTANCE BETWEEN LEFT AND RIGHT WHEELS (Meters)

// --- TABLE CONFIGURATION ---
// Starting at Bottom Right (X=2.0, Y=0.0) facing UP (90 degrees / PI/2)
#define START_X 3.0
#define START_Y 0.0
#define START_THETA (M_PI / 2.0) 

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// UDP CONFIG
#define SERVER_IP "127.0.0.1"
#define PORT 8888

int fd_spi;
int sock_fd;
struct sockaddr_in server_addr;

static int64_t now_ms() {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return (int64_t)ts.tv_sec * 1000LL + (int64_t)(ts.tv_nsec / 1000000LL);
}

// ==========================================
// SETUP FUNCTIONS
// ==========================================
void spi_init() {
    const char *device = "/dev/spidev0.0";
    if (access("/dev/spidev0.1", F_OK) == 0) device = "/dev/spidev0.1";
    
    fd_spi = open(device, O_RDWR);
    if (fd_spi < 0) { perror("Error opening SPI"); exit(1); }

    uint8_t mode = 0; uint8_t bits = 8; uint32_t speed = SPI_SPEED;
    ioctl(fd_spi, SPI_IOC_WR_MODE, &mode);
    ioctl(fd_spi, SPI_IOC_WR_BITS_PER_WORD, &bits);
    ioctl(fd_spi, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
}

void udp_init() {
    if ((sock_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("Socket creation failed");
        exit(EXIT_FAILURE);
    }
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(PORT);
    server_addr.sin_addr.s_addr = inet_addr(SERVER_IP);
}

int32_t read_fpga_register(uint8_t addr) {
    uint8_t tx1[] = {addr, 0, 0, 0}; uint8_t rx1[] = {0, 0, 0, 0}; 
    struct spi_ioc_transfer tr1 = { .tx_buf = (unsigned long)tx1, .rx_buf = (unsigned long)rx1, .len = 4, .speed_hz = SPI_SPEED, .bits_per_word = 8 };
    ioctl(fd_spi, SPI_IOC_MESSAGE(1), &tr1);

    uint8_t tx2[] = {0, 0, 0, 0}; uint8_t rx2[] = {0, 0, 0, 0};
    struct spi_ioc_transfer tr2 = { .tx_buf = (unsigned long)tx2, .rx_buf = (unsigned long)rx2, .len = 4, .speed_hz = SPI_SPEED, .bits_per_word = 8 };
    ioctl(fd_spi, SPI_IOC_MESSAGE(1), &tr2);

    return (rx2[0] << 24) | (rx2[1] << 16) | (rx2[2] << 8) | rx2[3];
}

// ==========================================
// MAIN LOOP
// ==========================================
int main() {
    spi_init();
    udp_init();

    printf("=== Robot Odometry Tracker Started ===\n");
    printf("Sending UDP data to localhost:%d\n", PORT);

    int32_t prev_odo_a = 0, prev_odo_b = 0;
    int first_run = 1;

    // Robot State
    double x = START_X;
    double y = START_Y;
    double theta = START_THETA;

    const double meters_per_tick = (2 * M_PI * WHEEL_RADIUS_M) / ODO_PPR;

    while(1) {
        // 1. Read Encoders
        int32_t odo_b = read_fpga_register(ADDR_ODO_A); // Right Wheel?
        int32_t odo_a = read_fpga_register(ADDR_ODO_B); // Left Wheel?

        odo_b = -odo_b; // Invert A as per your setup

        if (first_run) {
            prev_odo_a = odo_a;
            prev_odo_b = odo_b;
            first_run = 0;
            continue;
        }

        // 2. Calculate Delta Distances
        double d_right = (odo_a - prev_odo_a) * meters_per_tick;
        double d_left  = (odo_b - prev_odo_b) * meters_per_tick;

        prev_odo_a = odo_a;
        prev_odo_b = odo_b;

        // 3. Odometry Kinematics (Differential Drive)
        double d_center = (d_right + d_left) / 2.0;
        double d_theta  = (d_right - d_left) / WHEEL_BASE_M;

        // Update Pose
        theta += d_theta;
        
        // Normalize theta to -PI to PI (optional but good practice)
        if (theta > M_PI) theta -= 2*M_PI;
        if (theta < -M_PI) theta += 2*M_PI;

        x += d_center * cos(theta);
        y += d_center * sin(theta);

        // 4. Send Data via UDP (Format: "x,y,theta,timestamp_ms")
        char buffer[64];
        int64_t t_ms = now_ms();
        snprintf(buffer, sizeof(buffer), "%.4f,%.4f,%.4f,%lld", x, y, theta, (long long)t_ms);
        sendto(sock_fd, (const char *)buffer, strlen(buffer), 0, 
               (const struct sockaddr *)&server_addr, sizeof(server_addr));

        // 5. Console Debug (Optional)
        // printf("X: %.2f Y: %.2f Th: %.2f\r", x, y, theta);
        // fflush(stdout);

        usleep(20000); // 20ms update rate (50Hz)
    }

    close(fd_spi);
    close(sock_fd);
    return 0;
}

//gcc robot_odo.c -o robot_odo -lm