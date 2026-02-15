#include "SerialLidar.h"
#include "LidarSegmentation.h"
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <chrono>
#include <sstream>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

// Global state for visualization
struct VisState {
    std::vector<DetectedObject> objects;
    std::string mode_name;
    float canvas_size;
    int scan_count;
};

static double global_fps = 0.0;
static int frame_count = 0;
static auto last_fps_time = std::chrono::steady_clock::now();

static int64_t now_ms()
{
    auto now = std::chrono::system_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
}

static inline cv::Point2f lidarToRobot(float x_lidar, float y_lidar)
{
    // Calibrated from observed behavior:
    // real +Y appeared as -X, and real +X appeared as -Y.
    // Apply fixed frame remap so robot frame is consistent with global plotter.
    return cv::Point2f(-y_lidar, -x_lidar);
}

static std::string formatObjectsUdpMessage(const std::vector<DetectedObject>& objects,
                                           int64_t timestamp_ms)
{
    std::ostringstream oss;
    oss << timestamp_ms << "|";
    for (size_t i = 0; i < objects.size(); ++i) {
        cv::Point2f p_robot = lidarToRobot(objects[i].centroid_x, objects[i].centroid_y);
        if (i > 0) {
            oss << ";";
        }
        oss << std::fixed << std::setprecision(4)
            << p_robot.x << "," << p_robot.y;
    }
    return oss.str();
}

/**
 * Create real-time visualization Mat for continuous updates.
 */
Mat createVisualization(const VisState& state)
{
    int img_size = 800;
    Mat img(img_size, img_size, CV_8UC3, Scalar(255, 255, 255));
    
    float scale = img_size / (2.0f * state.canvas_size);
    auto worldToImage = [&](float x, float y) -> Point {
        int px = static_cast<int>(img_size / 2 + x * scale);
        int py = static_cast<int>(img_size / 2 - y * scale);
        return Point(px, py);
    };
    
    // Color palette for segments
    vector<Scalar> colors = {
        Scalar(255, 0, 0),      // Blue
        Scalar(0, 255, 0),      // Green
        Scalar(0, 0, 255),      // Red
        Scalar(255, 255, 0),    // Cyan
        Scalar(255, 0, 255),    // Magenta
        Scalar(0, 255, 255),    // Yellow
        Scalar(128, 0, 255),    // Purple
        Scalar(255, 128, 0),    // Orange
        Scalar(128, 255, 0),    // Lime
        Scalar(0, 128, 255),    // Deep Orange
    };
    
    int color_idx = 0;
    for (const auto& obj : state.objects) {
        Scalar color = colors[color_idx % colors.size()];
        
        // Draw segment points
        for (const auto& beam : obj.segment) {
            float x = beam.distance * std::cos(beam.angle);
            float y = beam.distance * std::sin(beam.angle);
            cv::Point2f p_robot = lidarToRobot(x, y);
            Point p = worldToImage(p_robot.x, p_robot.y);
            circle(img, p, 2, color, -1);
        }
        
        // Draw centroid
        cv::Point2f centroid_robot = lidarToRobot(obj.centroid_x, obj.centroid_y);
        Point centroid = worldToImage(centroid_robot.x, centroid_robot.y);
        circle(img, centroid, 5, color, 2);
        
        // Draw bounding box
        cv::Point2f c1 = lidarToRobot(obj.xmin, obj.ymin);
        cv::Point2f c2 = lidarToRobot(obj.xmax, obj.ymax);
        float bb_min_x = std::min(c1.x, c2.x);
        float bb_max_x = std::max(c1.x, c2.x);
        float bb_min_y = std::min(c1.y, c2.y);
        float bb_max_y = std::max(c1.y, c2.y);
        Point p_min = worldToImage(bb_min_x, bb_min_y);
        Point p_max = worldToImage(bb_max_x, bb_max_y);
        rectangle(img, p_min, p_max, color, 1);
        
        // Draw label
        string label = "Obj" + to_string(obj.segment_id);
        putText(img, label, centroid, FONT_HERSHEY_SIMPLEX, 0.4, color, 1);
        
        color_idx++;
    }
    
    // Draw origin and axes
    Point origin = worldToImage(0, 0);
    circle(img, origin, 3, Scalar(0, 0, 0), -1);
    line(img, origin, worldToImage(1.0, 0), Scalar(0, 0, 255), 2);  // X-axis (red)
    line(img, origin, worldToImage(0, 1.0), Scalar(0, 255, 0), 2);  // Y-axis (green)
    
    // Draw grid
    for (int i = -static_cast<int>(state.canvas_size); i <= static_cast<int>(state.canvas_size); ++i) {
        line(img, worldToImage(i, -state.canvas_size), worldToImage(i, state.canvas_size),
             Scalar(200, 200, 200), 1);
        line(img, worldToImage(-state.canvas_size, i), worldToImage(state.canvas_size, i),
             Scalar(200, 200, 200), 1);
    }
    
    // Add informational overlays
    putText(img, "Mode: " + state.mode_name, Point(10, 25), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 0), 2);
    putText(img, "Scan: " + to_string(state.scan_count), Point(10, 50), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 0), 2);
    putText(img, "Objects: " + to_string(state.objects.size()), Point(10, 75), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 0), 2);
    
    // FPS calculation
    frame_count++;
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_fps_time);
    if (elapsed.count() >= 500) {
        global_fps = frame_count * 1000.0 / elapsed.count();
        frame_count = 0;
        last_fps_time = now;
    }
    
    putText(img, "FPS: " + to_string(static_cast<int>(global_fps)), Point(10, 100), 
            FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 0), 2);
    
    return img;
}

/**
 * Print segment statistics to console.
 */
void printSegmentStats(int scan_num, const std::vector<DetectedObject>& objects, const string& mode)
{
    cout << "\n========== SCAN #" << scan_num << " (" << mode << ") ==========" << endl;
    if (objects.empty()) {
        cout << "No objects detected" << endl;
        return;
    }
    
    cout << left << setw(5) << "ID"
         << setw(8) << "Beams"
         << setw(15) << "Pos (X, Y)"
         << setw(12) << "Distance"
         << setw(12) << "Angle (°)"
         << setw(12) << "Quality" << endl;
    cout << string(90, '-') << endl;
    
    for (const auto& obj : objects) {
        cout << fixed << setprecision(2);
        cout << left << setw(5) << obj.segment_id
             << setw(8) << obj.segment.size();

           cv::Point2f p_robot = lidarToRobot(obj.centroid_x, obj.centroid_y);
        
        // Position
           cout << setw(15) << ("(" + to_string(p_robot.x).substr(0, 5) + "," + 
                           to_string(p_robot.y).substr(0, 5) + ")");
        
        // Distance and angle
        cout << setw(12) << obj.distance_to_centroid
             << setw(12) << (obj.angle_to_centroid * 180.0f / M_PI);
        
        // Average quality
        float avg_quality = 0;
        for (const auto& beam : obj.segment) {
            avg_quality += beam.quality;
        }
        avg_quality /= obj.segment.size();
        cout << setw(12) << static_cast<int>(avg_quality) << endl;
    }
}

int main(int argc, char** argv)
{
    cout << "\n╔════════════════════════════════════════════════════════════════╗" << endl;
    cout << "║     DetectEnemy C++ - Continuous LiDAR Object Detection       ║" << endl;
    cout << "║                Real-time 360° Rotation Scanning               ║" << endl;
    cout << "╚════════════════════════════════════════════════════════════════╝\n" << endl;
    
    // Configuration
    string port = "/dev/ttyUSB0";
    float min_dist_mm = 100.0f;
    float max_dist_mm = 4000.0f;
    bool use_linear_mode = true;
    
    // Parse command line arguments
    for (int i = 1; i < argc; ++i) {
        string arg = argv[i];
        if (arg == "--port" && i + 1 < argc) {
            port = argv[++i];
        } else if (arg == "--min-dist" && i + 1 < argc) {
            min_dist_mm = stof(argv[++i]);
        } else if (arg == "--max-dist" && i + 1 < argc) {
            max_dist_mm = stof(argv[++i]);
        } else if (arg == "--mode" && i + 1 < argc) {
            string mode_arg = argv[++i];
            use_linear_mode = (mode_arg == "linear");
        } else if (arg == "--help") {
            cout << "Usage: " << argv[0] << " [options]" << endl;
            cout << "  --port PORT           Serial port (default: /dev/ttyUSB0)" << endl;
            cout << "  --min-dist MM         Minimum distance in mm (default: 100)" << endl;
            cout << "  --max-dist MM         Maximum distance in mm (default: 4000)" << endl;
            cout << "  --mode MODE           'linear' or 'circle' (default: linear)" << endl;
            cout << "\nPress 'q' in visualization window to exit" << endl;
            return 0;
        }
    }
    
    cout << "[Config] Port: " << port << endl;
    cout << "[Config] Distance range: " << min_dist_mm << " - " << max_dist_mm << " mm" << endl;
    cout << "[Config] Mode: " << (use_linear_mode ? "Linear Threshold" : "Exact Circle") << endl;
    cout << "\nWaiting for LIDAR connection..." << endl;
    
    // Connect to LIDAR
    SerialLidar lidar(port, 115200, min_dist_mm, max_dist_mm);
    if (!lidar.connect()) {
        cerr << "\n[ERROR] Failed to connect to LIDAR at " << port << endl;
        cerr << "Please verify:" << endl;
        cerr << "  1. Device is connected to " << port << endl;
        cerr << "  2. Run: sudo usermod -a -G dialout $USER" << endl;
        cerr << "  3. Log out and back in" << endl;
        return 1;
    }

    // Setup UDP publisher for global plotter objects stream
    int obj_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (obj_sock < 0) {
        cerr << "\n[ERROR] Failed to create UDP socket for object publishing" << endl;
        lidar.disconnect();
        return 1;
    }

    sockaddr_in obj_addr;
    memset(&obj_addr, 0, sizeof(obj_addr));
    obj_addr.sin_family = AF_INET;
    obj_addr.sin_port = htons(8890);
    obj_addr.sin_addr.s_addr = inet_addr("127.0.0.1");

    cout << "[UDP] Publishing object centroids to 127.0.0.1:8890" << endl;
    
    // Select segmentation mode
    LidarSegmentation::Mode seg_mode = use_linear_mode 
        ? LidarSegmentation::LINEAR 
        : LidarSegmentation::EXACT_CIRCLE;
    
    string window_title = use_linear_mode 
        ? "DetectEnemy - Linear Mode (Press 'q' to quit)"
        : "DetectEnemy - Circle Mode (Press 'q' to quit)";
    
    namedWindow(window_title, WINDOW_NORMAL);
    resizeWindow(window_title, 800, 800);
    
    int scan_count = 0;
    bool running = true;
    
    cout << "\n[Ready] Starting continuous scanning. Window will update in real-time..." << endl;
    cout << "[Info] Close window or press 'q' to exit." << endl << endl;
    
    try {
        // Continuous scanning loop
        while (running) {
            auto start_time = chrono::high_resolution_clock::now();
            
            // Read one complete 360° rotation
            auto scan = lidar.readScan();
            if (scan.empty()) {
                cerr << "\n[WARNING] No scan data received. Retrying..." << endl;
                continue;
            }
            
            scan_count++;
            
            // Compute uniform angular spacing
            float uniform_delta_theta = 0.0f;
            if (scan.size() > 1) {
                uniform_delta_theta = (2.36f - (-2.36f)) / (scan.size() - 1);
            }
            
            // Perform segmentation
            std::vector<DetectedObject> objects;
            
            if (seg_mode == LidarSegmentation::LINEAR) {
                auto segments = LidarSegmentation::segment(
                    scan,
                    LidarSegmentation::LINEAR,
                    3,           // min_segment_len
                    0.08f,       // a
                    0.015f,      // b
                    0.0f,        // R (unused)
                    uniform_delta_theta,
                    false        // connect_wrap
                );
                objects = LidarSegmentation::segmentsToObjects(segments);
            } else {
                auto segments = LidarSegmentation::segment(
                    scan,
                    LidarSegmentation::EXACT_CIRCLE,
                    3,           // min_segment_len
                    0.0f,        // a (unused)
                    0.0f,        // b (unused)
                    0.50f,       // R (circle radius)
                    uniform_delta_theta,
                    false        // connect_wrap
                );
                objects = LidarSegmentation::segmentsToObjects(segments);
            }

                 int64_t objects_t_ms = now_ms();
                 std::string udp_payload = formatObjectsUdpMessage(objects, objects_t_ms);
                 sendto(obj_sock,
                     udp_payload.c_str(),
                     udp_payload.size(),
                     0,
                     reinterpret_cast<const sockaddr*>(&obj_addr),
                     sizeof(obj_addr));
            
            // Print statistics
            string mode_str = use_linear_mode ? "LINEAR" : "CIRCLE";
            printSegmentStats(scan_count, objects, mode_str);
            
            // Create and display visualization
            VisState state;
            state.objects = objects;
            state.mode_name = mode_str;
            state.canvas_size = 5.0f;
            state.scan_count = scan_count;
            
            Mat img = createVisualization(state);
            imshow(window_title, img);
            
            // Check for user input (q to quit, or ESC)
            int key = waitKey(1) & 0xFF;
            if (key == 'q' || key == 'Q' || key == 27) {  // 27 is ESC
                running = false;
                cout << "\n[Info] User exit requested" << endl;
            }
            
            // Timing information
            auto end_time = chrono::high_resolution_clock::now();
            auto duration = chrono::duration_cast<chrono::milliseconds>(end_time - start_time);
            
            cout << "[Timing] Scan cycle: " << duration.count() << "ms | "
                 << "Beams: " << scan.size() << " | "
                 << "Objects: " << objects.size() << endl;
        }
        
    } catch (const exception& e) {
        cerr << "\n[ERROR] Exception: " << e.what() << endl;
        close(obj_sock);
        lidar.disconnect();
        destroyAllWindows();
        return 1;
    }
    
    // Cleanup
    close(obj_sock);
    lidar.disconnect();
    destroyAllWindows();
    
    cout << "\n╔════════════════════════════════════════════════════════════════╗" << endl;
    cout << "║                    Scanning Complete                          ║" << endl;
    cout << "║                 Total Scans: " << setw(40) << scan_count << " ║" << endl;
    cout << "╚════════════════════════════════════════════════════════════════╝\n" << endl;
    
    return 0;
}