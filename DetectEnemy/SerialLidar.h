#pragma once

#include <cstdint>
#include <vector>
#include <string>
#include <memory>

/**
 * Represents a single LIDAR beam reading.
 */
struct Beam
{
    float distance;  // meters
    float angle;     // radians
    uint8_t quality; // reflectivity (0-63)
    
    Beam(float dist = 0.0f, float ang = 0.0f, uint8_t qual = 0)
        : distance(dist), angle(ang), quality(qual) {}
};

/**
 * Serial communication interface for RPLIDAR devices.
 * Handles port setup, motor control, and packet decoding.
 */
class SerialLidar
{
public:
    SerialLidar(const std::string& port = "/dev/ttyUSB0", 
                int baudrate = 115200,
                float min_dist_mm = 100.0f,
                float max_dist_mm = 4000.0f);
    
    ~SerialLidar();
    
    /**
     * Connect to the LIDAR device and start the motor.
     * @return true if successful, false otherwise
     */
    bool connect();
    
    /**
     * Disconnect from the device and stop the motor.
     */
    void disconnect();
    
    /**
     * Read a complete 360-degree scan.
     * @return Vector of Beam objects from one full rotation
     */
    std::vector<Beam> readScan();
    
    /**
     * Check if connected to device.
     * @return true if connected
     */
    bool isConnected() const { return fd_ != -1; }
    
private:
    std::string port_;
    int baudrate_;
    int fd_;  // file descriptor
    
    float min_dist_mm_;
    float max_dist_mm_;
    
    // Serial port setup
    bool openPort_();
    void closePort_();
    
    // Motor control (RPLIDAR specific)
    void setDTR_(bool on);
    
    // Packet processing
    bool readPacket_(Beam& beam);
    
    // Helper: read exactly n bytes
    int readBytes_(uint8_t* buffer, int n);
};
