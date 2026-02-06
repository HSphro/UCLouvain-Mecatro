#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

// ==========================================
// CONFIGURATION
// ==========================================
#define SPI_SPEED 500000  // 500 kHz

// FPGA Addresses
#define ADDR_ODO_A 0x10
#define ADDR_ODO_B 0x20
#define ADDR_ENC_A 0x13
#define ADDR_ENC_B 0x23

int fd_spi;

// ==========================================
// SPI SETUP
// ==========================================
void spi_init() {
    // 1. Auto-Detect SPI Device (CE0 or CE1)
    const char *device = "/dev/spidev0.0";
    if (access("/dev/spidev0.1", F_OK) == 0) {
        device = "/dev/spidev0.1";
        printf("Auto-detected SPI on CE1 (%s)\n", device);
    } else {
        printf("Auto-detected SPI on CE0 (%s)\n", device);
    }

    // 2. Open File Descriptor
    fd_spi = open(device, O_RDWR);
    if (fd_spi < 0) {
        perror("Error opening SPI Device");
        printf("Make sure SPI is enabled in 'sudo raspi-config'\n");
        exit(1);
    }

    // 3. Configure Mode and Speed
    uint8_t mode = 0;    // Mode 0 (CPOL=0, CPHA=0)
    uint8_t bits = 8;
    uint32_t speed = SPI_SPEED;

    if (ioctl(fd_spi, SPI_IOC_WR_MODE, &mode) < 0) perror("Failed to set Mode");
    if (ioctl(fd_spi, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) perror("Failed to set Bits");
    if (ioctl(fd_spi, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) perror("Failed to set Speed");
}

// ==========================================
// READ FUNCTION
// ==========================================
int32_t read_fpga_register(uint8_t addr) {
    // Transaction 1: Send Address (FPGA prepares data)
    // We send [Addr, 0, 0, 0]
    uint8_t tx1[] = {addr, 0, 0, 0};
    uint8_t rx1[] = {0, 0, 0, 0}; // Response ignored for first step
    
    struct spi_ioc_transfer tr1 = {
        .tx_buf = (unsigned long)tx1,
        .rx_buf = (unsigned long)rx1,
        .len = 4,
        .speed_hz = SPI_SPEED,
        .bits_per_word = 8,
    };
    
    // Perform transfer
    if (ioctl(fd_spi, SPI_IOC_MESSAGE(1), &tr1) < 0) {
        perror("SPI Transfer Failed");
        return 0;
    }

    // Short delay is sometimes helpful for FPGA logic stability, 
    // though usually not needed in C due to syscall overhead.
    // usleep(10); 

    // Transaction 2: Read Data
    // We send dummy zeros to clock out the data
    uint8_t tx2[] = {0, 0, 0, 0};
    uint8_t rx2[] = {0, 0, 0, 0};

    struct spi_ioc_transfer tr2 = {
        .tx_buf = (unsigned long)tx2,
        .rx_buf = (unsigned long)rx2,
        .len = 4,
        .speed_hz = SPI_SPEED,
        .bits_per_word = 8,
    };

    if (ioctl(fd_spi, SPI_IOC_MESSAGE(1), &tr2) < 0) {
        perror("SPI Read Failed");
        return 0;
    }

    // Convert 4 bytes (Big Endian) to 32-bit Signed Integer
    // We cast to int8_t first to handle sign extension correctly if needed,
    // but standard shifting works fine for 32-bit reconstruction.
    int32_t val = (rx2[0] << 24) | (rx2[1] << 16) | (rx2[2] << 8) | rx2[3];

    return val;
}

// ==========================================
// MAIN LOOP
// ==========================================
int main() {
    spi_init();

    printf("\nStarting C Sensor Reader...\n");
    printf("Press Ctrl+C to stop.\n");
    printf("----------------------------------------------------------------\n");
    printf("%-12s | %-12s | %-12s | %-12s\n", "ODO A", "ODO B", "ENC A", "ENC B");
    printf("----------------------------------------------------------------\n");

    while(1) {
        // Read all 4 registers
        int32_t odo_a = read_fpga_register(ADDR_ODO_A);
        int32_t odo_b = read_fpga_register(ADDR_ODO_B);
        int32_t enc_a = read_fpga_register(ADDR_ENC_A);
        int32_t enc_b = read_fpga_register(ADDR_ENC_B);

        // Print using \r to overwrite the line
        // %-12d means left-aligned integer in 12-char space
        printf("\r%-12d | %-12d | %-12d | %-12d", odo_a, odo_b, enc_a, enc_b);
        
        // Flush stdout so the text appears immediately
        fflush(stdout);

        // Sleep 50ms (20Hz update rate)
        usleep(50000);
    }

    close(fd_spi);
    return 0;
}