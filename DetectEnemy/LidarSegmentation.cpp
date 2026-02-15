#include "LidarSegmentation.h"
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <sstream>

DetectedObject::DetectedObject(const std::vector<Beam>& seg, int id)
    : segment_id(id), segment(seg)
{
    if (segment.empty()) {
        centroid_x = centroid_y = 0.0f;
        distance_to_centroid = 0.0f;
        angle_to_centroid = 0.0f;
        xmin = xmax = ymin = ymax = 0.0f;
        width = height = 0.0f;
        return;
    }
    
    // Compute centroid
    float sum_x = 0.0f, sum_y = 0.0f;
    float min_x = 1e6f, max_x = -1e6f;
    float min_y = 1e6f, max_y = -1e6f;
    
    for (const auto& beam : segment) {
        float x = polarToCartesianX(beam.distance, beam.angle);
        float y = polarToCartesianY(beam.distance, beam.angle);
        sum_x += x;
        sum_y += y;
        min_x = std::min(min_x, x);
        max_x = std::max(max_x, x);
        min_y = std::min(min_y, y);
        max_y = std::max(max_y, y);
    }
    
    centroid_x = sum_x / segment.size();
    centroid_y = sum_y / segment.size();
    xmin = min_x;
    xmax = max_x;
    ymin = min_y;
    ymax = max_y;
    width = xmax - xmin;
    height = ymax - ymin;
    
    // Distance and angle to centroid
    distance_to_centroid = std::sqrt(centroid_x * centroid_x + centroid_y * centroid_y);
    angle_to_centroid = std::atan2(centroid_y, centroid_x);
}

std::string DetectedObject::toString() const
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2);
    oss << "DetectedObject(id=" << segment_id
        << ", pos=(" << centroid_x << ", " << centroid_y << ")"
        << ", dist=" << distance_to_centroid << "m"
        << ", angle=" << (angle_to_centroid * 180.0f / M_PI) << "°"
        << ", size=" << width << "×" << height << "m"
        << ", beams=" << segment.size() << ")";
    return oss.str();
}

bool LidarSegmentation::sameSegment_(float r0, float r1, float alpha, Mode mode,
                                      float a, float b, float R)
{
    // Guard invalid numerics
    if (!std::isfinite(r0) || !std::isfinite(r1) || r0 <= 0.0f || r1 <= 0.0f) {
        return false;
    }
    
    if (mode == LINEAR) {
        float thr = a + b * std::min(r0, r1);
        return std::abs(r1 - r0) <= thr;
    }
    else if (mode == EXACT_CIRCLE) {
        // Circle model: σ = sqrt(max(0, 4R² - r0² sin²(α)))
        float mu = r0 * (std::cos(alpha) - 1.0f);
        float under = 4.0f * (R * R) - (r0 * r0) * (std::sin(alpha) * std::sin(alpha));
        if (under < 0.0f) {
            return false;  // Geometrically impossible
        }
        float sigma = std::sqrt(under);
        return std::abs((r1 - r0) - mu) <= sigma;
    }
    
    return false;
}

std::vector<std::vector<Beam>> LidarSegmentation::segment(
    const std::vector<Beam>& scan,
    Mode mode,
    int min_segment_len,
    float a, float b, float R,
    float uniform_delta_theta,
    bool connect_wrap)
{
    if (scan.empty()) {
        return {};
    }
    
    std::vector<std::vector<Beam>> segments;
    std::vector<Beam> current;
    current.push_back(scan[0]);
    
    size_t n = scan.size();
    
    // Main segmentation loop
    for (size_t i = 0; i < n - 1; ++i) {
        float r0 = scan[i].distance;
        float theta0 = scan[i].angle;
        float r1 = scan[i + 1].distance;
        float theta1 = scan[i + 1].angle;
        
        float alpha = (uniform_delta_theta != 0.0f)
                        ? uniform_delta_theta
                        : (theta1 - theta0);
        
        if (sameSegment_(r0, r1, alpha, mode, a, b, R)) {
            current.push_back(scan[i + 1]);
        } else {
            if ((int)current.size() >= min_segment_len) {
                segments.push_back(current);
            }
            current.clear();
            current.push_back(scan[i + 1]);
        }
    }
    
    // Append last segment
    if ((int)current.size() >= min_segment_len) {
        segments.push_back(current);
    }
    
    // Optional wrap-around connection
    if (connect_wrap && segments.size() >= 2) {
        auto& first_seg = segments.front();
        auto& last_seg = segments.back();
        
        float r0 = last_seg.back().distance;
        float theta0 = last_seg.back().angle;
        float r1 = first_seg.front().distance;
        float theta1 = first_seg.front().angle;
        
        float alpha_wrap = (uniform_delta_theta != 0.0f)
                            ? uniform_delta_theta
                            : ((theta1 + 2.0f * M_PI) - theta0);
        
        if (sameSegment_(r0, r1, alpha_wrap, mode, a, b, R)) {
            // Merge
            last_seg.insert(last_seg.end(), first_seg.begin(), first_seg.end());
            segments.erase(segments.begin());
        }
    }
    
    return segments;
}

std::vector<DetectedObject> LidarSegmentation::segmentsToObjects(
    const std::vector<std::vector<Beam>>& segments)
{
    std::vector<DetectedObject> objects;
    for (size_t i = 0; i < segments.size(); ++i) {
        objects.emplace_back(segments[i], static_cast<int>(i));
    }
    return objects;
}
