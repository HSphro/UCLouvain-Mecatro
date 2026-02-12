#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <sys/ioctl.h>

// ==========================================
// CONFIGURATION
// ==========================================
// Check your /dev/ folder. It might be ttyUSB0, ttyUSB1, or ttyACM0
#define PORT_NAME "/dev/ttyUSB0" 
#define BAUDRATE  B115200

// Filter range (in millimeters)
#define MIN_DIST_MM  200.0   
#define MAX_DIST_MM  1000.0  

// ==========================================
// SERIAL SETUP
// ==========================================
int open_serial_port(const char *device) {
    int fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        perror("ERROR: Could not open port");
        exit(1);
    }

    struct termios options;
    tcgetattr(fd, &options);
    
    // Set Baud Rate
    cfsetispeed(&options, BAUDRATE);
    cfsetospeed(&options, BAUDRATE);
    
    // 8N1 Configuration (8 data bits, No parity, 1 stop bit)
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag |= (CLOCAL | CREAD);

    // Raw Mode (Disable processing)
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;

    tcsetattr(fd, TCSANOW, &options);
    fcntl(fd, F_SETFL, 0); 
    return fd;
}

// Control DTR to spin the motor (RPLIDAR A1/A2 specific)
void set_dtr(int fd, int on) {
    int status;
    ioctl(fd, TIOCMGET, &status);
    if (on) status &= ~TIOCM_DTR; // Low DTR enables motor via PNP transistor often used on adapters
    else    status |= TIOCM_DTR;  
    ioctl(fd, TIOCMSET, &status);
}

// ==========================================
// MAIN LOOP
// ==========================================
int main() {
    // Disable stdout buffering so printf appears immediately
    setbuf(stdout, NULL); 

    printf("--- Opening Serial Port %s ---\n", PORT_NAME);
    int fd = open_serial_port(PORT_NAME);

    printf("--- Starting Motor ---\n");
    set_dtr(fd, 1); 
    usleep(200000); // Wait for spin-up

    // Send Scan Command
    unsigned char cmd[] = {0xA5, 0x20}; 
    if (write(fd, cmd, 2) != 2) {
        perror("Error writing command");
        close(fd);
        return 1;
    }

    printf("--- Filtering: %.0fmm to %.0fmm ---\n", MIN_DIST_MM, MAX_DIST_MM);

    unsigned char byte;
    unsigned char buffer[5];
    int idx = 0;

    // Flush any garbage data currently in buffer
    tcflush(fd, TCIFLUSH);

    while (1) {
        int n = read(fd, &byte, 1);
        
        if (n > 0) {
            // --- STATE 0: Sync Header (Byte 0) ---
            // The protocol requires: (bit 0) XOR (bit 1) == 1
            // This detects the "S" and "!S" bits.
            if (idx == 0) {
                if ( ((byte & 0x01) ^ ((byte >> 1) & 0x01)) ) {
                    buffer[0] = byte;
                    idx++;
                } else {
                    idx = 0; // Drop byte, keep searching for sync
                }
            } 
            // --- STATE 1: Check Bit (Byte 1) ---
            // The lowest bit of Byte 1 is always 1
            else if (idx == 1) {
                if ((byte & 0x01) == 1) {
                    buffer[1] = byte;
                    idx++;
                } else {
                    idx = 0; // Check failed, reset sync
                }
            }
            // --- STATE 2-4: Read Payload ---
            else {
                buffer[idx++] = byte;

                if (idx == 5) {
                    // --- PACKET COMPLETE: DECODE ---
                    
                    // Quality/Reflectivity in Byte 0 bits 7-2 (6 bits)
                    // Ranges 0-63; higher = better reflectivity
                    int quality = (buffer[0] >> 2) & 0x3F;
                    
                    // Angle Formula:
                    // 1. Shift Byte 1 right by 1 to remove the check bit
                    // 2. Shift Byte 2 left by 7 
                    // 3. Divide by 64.0
                    float angle = ((buffer[1] >> 1) | (buffer[2] << 7)) / 64.0;
                    
                    // Distance Formula:
                    // 1. Combine Byte 3 and Byte 4
                    // 2. Divide by 4.0
                    float dist  = ((buffer[3]) | (buffer[4] << 8)) / 4.0;

                    // Normalize Angle (0-360)
                    if (angle >= 360.0) angle -= 360.0;
                    if (angle < 0.0) angle += 360.0;

                    // --- FILTERING ---
                    
                    // Filter 1: Valid Reading (dist > 0)
                    // RPLIDAR sends 0 if it sees nothing or failed to measure
                    if (dist > 0) {
                        printf("Angle: %6.2f deg   Distance: %6.2f mm   Quality: %d\n", angle, dist, quality);
                    }

                    // Reset for next packet
                    idx = 0;
                }
            }
        } 
    }

    set_dtr(fd, 0); // Stop motor
    close(fd);
    return 0;
}