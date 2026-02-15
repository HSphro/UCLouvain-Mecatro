#include "SerialLidar.h"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <cstring>
#include <iostream>
#include <cmath>
#include <stdexcept>

SerialLidar::SerialLidar(const std::string& port, int baudrate,
                         float min_dist_mm, float max_dist_mm)
    : port_(port), baudrate_(baudrate), fd_(-1),
      min_dist_mm_(min_dist_mm), max_dist_mm_(max_dist_mm)
{
}

SerialLidar::~SerialLidar()
{
    disconnect();
}

bool SerialLidar::connect()
{
    if (!openPort_()) {
        std::cerr << "Failed to open serial port: " << port_ << std::endl;
        return false;
    }
    
    // Give the device a moment to initialize
    usleep(200000);
    
    // Start motor (DTR low enables via PNP transistor)
    setDTR_(true);
    usleep(200000);
    
    // Send scan command (0xA5 0x20)
    uint8_t cmd[] = {0xA5, 0x20};
    if (write(fd_, cmd, 2) != 2) {
        std::cerr << "Failed to send scan command" << std::endl;
        closePort_();
        return false;
    }
    
    // Flush any garbage data
    tcflush(fd_, TCIFLUSH);
    
    std::cout << "[SerialLidar] Connected to " << port_ << std::endl;
    return true;
}

void SerialLidar::disconnect()
{
    if (fd_ != -1) {
        setDTR_(false);  // Stop motor
        closePort_();
    }
}

bool SerialLidar::openPort_()
{
    fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd_ == -1) {
        return false;
    }
    
    struct termios options;
    tcgetattr(fd_, &options);
    
    // Set baud rate
    speed_t baud_code;
    switch (baudrate_) {
        case 115200: baud_code = B115200; break;
        case 57600:  baud_code = B57600;  break;
        case 9600:   baud_code = B9600;   break;
        default:     baud_code = B115200; break;
    }
    
    cfsetispeed(&options, baud_code);
    cfsetospeed(&options, baud_code);
    
    // 8N1 configuration
    options.c_cflag &= ~PARENB;     // No parity
    options.c_cflag &= ~CSTOPB;     // 1 stop bit
    options.c_cflag &= ~CSIZE;      // Clear size
    options.c_cflag |= CS8;         // 8 data bits
    options.c_cflag |= (CLOCAL | CREAD);  // Enable read/write
    
    // Raw mode
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;
    
    tcsetattr(fd_, TCSANOW, &options);
    fcntl(fd_, F_SETFL, 0);
    
    return true;
}

void SerialLidar::closePort_()
{
    if (fd_ != -1) {
        close(fd_);
        fd_ = -1;
    }
}

void SerialLidar::setDTR_(bool on)
{
    if (fd_ == -1) return;
    
    int status;
    ioctl(fd_, TIOCMGET, &status);
    if (on) {
        status &= ~TIOCM_DTR;  // Clear DTR (low enables motor)
    } else {
        status |= TIOCM_DTR;   // Set DTR (high disables motor)
    }
    ioctl(fd_, TIOCMSET, &status);
}

int SerialLidar::readBytes_(uint8_t* buffer, int n)
{
    int total = 0;
    while (total < n) {
        int n_read = read(fd_, buffer + total, n - total);
        if (n_read <= 0) {
            usleep(1000);  // Small delay to avoid busy-waiting
            continue;
        }
        total += n_read;
    }
    return total;
}

bool SerialLidar::readPacket_(Beam& beam)
{
    static uint8_t buffer[5];
    static int idx = 0;
    uint8_t byte;
    
    while (true) {
        if (read(fd_, &byte, 1) <= 0) {
            usleep(1000);
            continue;
        }
        
        // STATE 0: Sync header
        // Bit check: (bit 0) XOR (bit 1) == 1
        if (idx == 0) {
            if (((byte & 0x01) ^ ((byte >> 1) & 0x01)) == 1) {
                buffer[0] = byte;
                idx++;
            } else {
                idx = 0;
            }
        }
        // STATE 1: Check byte (LSB must be 1)
        else if (idx == 1) {
            if ((byte & 0x01) == 1) {
                buffer[1] = byte;
                idx++;
            } else {
                idx = 0;
            }
        }
        // STATE 2-4: Payload
        else {
            buffer[idx++] = byte;
            
            if (idx == 5) {
                // Decode packet
                uint8_t quality = (buffer[0] >> 2) & 0x3F;
                
                // Angle: (byte1 >> 1) | (byte2 << 7), divided by 64, in degrees
                float angle_deg = ((buffer[1] >> 1) | (buffer[2] << 7)) / 64.0f;
                if (angle_deg >= 360.0f) angle_deg -= 360.0f;
                if (angle_deg < 0.0f) angle_deg += 360.0f;
                
                // Distance: (byte3) | (byte4 << 8), divided by 4, in mm
                float dist_mm = ((buffer[3]) | (buffer[4] << 8)) / 4.0f;
                
                idx = 0;  // Reset for next packet
                
                // Filter by distance range
                if (dist_mm > 0 && dist_mm >= min_dist_mm_ && dist_mm <= max_dist_mm_) {
                    // Convert to radians and meters
                    float angle_rad = angle_deg * M_PI / 180.0f;
                    float dist_m = dist_mm / 1000.0f;
                    
                    beam = Beam(dist_m, angle_rad, quality);
                    return true;
                }
                // If out of range, keep reading
            }
        }
    }
}

std::vector<Beam> SerialLidar::readScan()
{
    if (!isConnected()) {
        std::cerr << "Not connected to LIDAR" << std::endl;
        return {};
    }
    
    std::vector<Beam> scan;
    float last_angle = -1.0f;
    
    // Read beams until we complete a full rotation (angle wraps from ~2π to ~0)
    while (true) {
        Beam beam;
        if (readPacket_(beam)) {
            // Detect wrap-around (indicates end of rotation)
            if (last_angle > 3.0f && beam.angle < 1.0f) {
                scan.push_back(beam);
                break;  // End of scan
            }
            scan.push_back(beam);
            last_angle = beam.angle;
        }
    }
    
    std::cout << "[SerialLidar] Read " << scan.size() << " beams" << std::endl;
    return scan;
}
