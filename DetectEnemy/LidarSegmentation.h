#pragma once

#include "SerialLidar.h"
#include <vector>
#include <cmath>

/**
 * Represents a detected object from LiDAR segmentation.
 */
struct DetectedObject
{
    int segment_id;
    std::vector<Beam> segment;
    
    // Derived properties
    float centroid_x;
    float centroid_y;
    float distance_to_centroid;
    float angle_to_centroid;
    
    float xmin, xmax, ymin, ymax;
    float width, height;
    
    DetectedObject() = default;
    explicit DetectedObject(const std::vector<Beam>& seg, int id);
    
    std::string toString() const;
};

/**
 * Range-jump segmentation for LiDAR point cloud.
 * Implements both linear and exact circular model approaches.
 */
class LidarSegmentation
{
public:
    enum Mode {
        LINEAR = 1,        // |Δr| ≤ a + b * min(r0, r1)
        EXACT_CIRCLE = 2   // |(Δr) - μ| ≤ σ with circle geometry
    };
    
    /**
     * Segment a 2D LiDAR scan into contiguous segments.
     * 
     * @param scan Vector of Beam readings
     * @param mode Segmentation mode (LINEAR or EXACT_CIRCLE)
     * @param min_segment_len Minimum beams per segment
     * @param a Linear threshold parameter (mode LINEAR)
     * @param b Linear threshold parameter (mode LINEAR)
     * @param R Circle radius in meters (mode EXACT_CIRCLE)
     * @param uniform_delta_theta Fixed angular spacing (if known)
     * @param connect_wrap Connect wrap-around boundary
     * @return Vector of segments, each a vector of Beam
     */
    static std::vector<std::vector<Beam>> segment(
        const std::vector<Beam>& scan,
        Mode mode = LINEAR,
        int min_segment_len = 2,
        float a = 0.10f,
        float b = 0.01f,
        float R = 0.50f,
        float uniform_delta_theta = 0.0f,
        bool connect_wrap = false);
    
    /**
     * Convert segments to DetectedObject representation.
     * 
     * @param segments Vector of segments
     * @return Vector of DetectedObject
     */
    static std::vector<DetectedObject> segmentsToObjects(
        const std::vector<std::vector<Beam>>& segments);
    
private:
    /**
     * Check if two adjacent beams belong to the same segment.
     * 
     * @param r0 First beam distance (meters)
     * @param r1 Second beam distance (meters)
     * @param alpha Angular spacing between beams (radians)
     * @param mode Segmentation mode
     * @param a Parameter for LINEAR mode
     * @param b Parameter for LINEAR mode
     * @param R Parameter for EXACT_CIRCLE mode
     * @return true if same segment
     */
    static bool sameSegment_(float r0, float r1, float alpha, Mode mode,
                             float a, float b, float R);
};

// Utility functions
inline float polarToCartesianX(float r, float theta)
{
    return r * std::cos(theta);
}

inline float polarToCartesianY(float r, float theta)
{
    return r * std::sin(theta);
}

inline float angleNormalize(float theta)
{
    return std::atan2(std::sin(theta), std::cos(theta));
}
