#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <sys/ioctl.h>

#define PORT_NAME "/dev/ttyUSB0"
#define BAUDRATE  B115200

// ==========================================
// CONFIGURATION (Change these)
// ==========================================
#define MIN_DIST_MM  200.0  // 20 cm
#define MAX_DIST_MM  500.0  // 30 cm

// ==========================================
// SETUP
// ==========================================
int open_serial_port(const char *device) {
    int fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        perror("ERROR: Could not open port");
        exit(1);
    }

    struct termios options;
    tcgetattr(fd, &options);
    cfsetispeed(&options, BAUDRATE);
    cfsetospeed(&options, BAUDRATE);
    
    // 8N1
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag |= (CLOCAL | CREAD);

    // Raw Mode
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;

    tcsetattr(fd, TCSANOW, &options);
    fcntl(fd, F_SETFL, 0); 
    return fd;
}

void set_dtr(int fd, int on) {
    int status;
    ioctl(fd, TIOCMGET, &status);
    if (on) status &= ~TIOCM_DTR; 
    else    status |= TIOCM_DTR;  
    ioctl(fd, TIOCMSET, &status);
}

// ==========================================
// MAIN LOOP
// ==========================================
int main() {
    setbuf(stdout, NULL); 

    printf("--- Opening Serial Port ---\n");
    int fd = open_serial_port(PORT_NAME);

    printf("--- Starting Motor ---\n");
    set_dtr(fd, 1); 
    usleep(100000);

    unsigned char cmd[] = {0xA5, 0x20}; 
    write(fd, cmd, 2);

    printf("--- Filtering: %.0fmm to %.0fmm ---\n", MIN_DIST_MM, MAX_DIST_MM);

    unsigned char byte;
    unsigned char buffer[5];
    int idx = 0;

    while (1) {
        int n = read(fd, &byte, 1);
        
        if (n > 0) {
            if (idx == 0) {
                if ((byte & 0x03) == 0x01 && (byte >> 2) != 0) {
                    buffer[0] = byte;
                    idx++;
                }
            } 
            else {
                buffer[idx++] = byte;

                if (idx == 5) {
                    float angle = ((buffer[1] >> 1) | (buffer[2] << 7)) / 64.0;
                    float dist  = ((buffer[3]) | (buffer[4] << 8)) / 4.0;

                    // Normalize Angle
                    while (angle >= 360.0) angle -= 360.0;
                    if (angle < 0.0) angle += 360.0;

                    // FILTER: Check Min and Max Radius
                    if (dist >= MIN_DIST_MM && dist <= MAX_DIST_MM) {
                        printf("Angle: %6.2f deg   Distance: %6.2f mm\n", angle, dist);
                    }

                    idx = 0;
                }
            }
        } 
    }

    close(fd);
    return 0;
}