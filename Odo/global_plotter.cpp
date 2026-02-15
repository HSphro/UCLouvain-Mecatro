#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cmath>
#include <chrono>
#include <cstring>
#include <cstdlib>   // <-- needed for setenv
#include <deque>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

struct Pose2D {
    double x;
    double y;
    double theta;
};

struct Object2D {
    double x;
    double y;
};

struct TimedPose {
    Pose2D pose;
    int64_t timestamp_ms;
};

struct ParsedObjects {
    std::vector<Object2D> objects;
    int64_t timestamp_ms;
    bool has_timestamp;
};

static int64_t now_ms() {
    auto now = std::chrono::system_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
}

static double normalize_angle(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

static Pose2D pose_at_time(const std::deque<TimedPose>& history,
                           int64_t target_ms,
                           int64_t& used_timestamp_ms) {
    if (history.empty()) {
        used_timestamp_ms = target_ms;
        return Pose2D{0.0, 0.0, 0.0};
    }

    if (target_ms <= history.front().timestamp_ms) {
        used_timestamp_ms = history.front().timestamp_ms;
        return history.front().pose;
    }

    if (target_ms >= history.back().timestamp_ms) {
        used_timestamp_ms = history.back().timestamp_ms;
        return history.back().pose;
    }

    for (size_t i = 1; i < history.size(); ++i) {
        const auto& p0 = history[i - 1];
        const auto& p1 = history[i];
        if (target_ms <= p1.timestamp_ms) {
            const double dt = static_cast<double>(p1.timestamp_ms - p0.timestamp_ms);
            const double alpha = (dt > 0.0)
                                     ? static_cast<double>(target_ms - p0.timestamp_ms) / dt
                                     : 0.0;

            Pose2D out;
            out.x = p0.pose.x + alpha * (p1.pose.x - p0.pose.x);
            out.y = p0.pose.y + alpha * (p1.pose.y - p0.pose.y);
            double dtheta = normalize_angle(p1.pose.theta - p0.pose.theta);
            out.theta = normalize_angle(p0.pose.theta + alpha * dtheta);
            used_timestamp_ms = target_ms;
            return out;
        }
    }

    used_timestamp_ms = history.back().timestamp_ms;
    return history.back().pose;
}

static int make_udp_socket(int port) {
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        perror("socket");
        return -1;
    }

    // Allow quick reuse of port (helps during development)
    int opt = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(port));
    addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        perror("bind");
        close(sock);
        return -1;
    }

    int flags = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, flags | O_NONBLOCK);

    return sock;
}

static bool parse_pose(const std::string& msg,
                       Pose2D& pose,
                       int64_t& timestamp_ms,
                       bool& has_timestamp) {
    std::stringstream ss(msg);
    std::string token;
    std::vector<double> values;

    while (std::getline(ss, token, ',')) {
        try {
            values.push_back(std::stod(token));
        } catch (...) {
            return false;
        }
    }

    if (values.size() != 3 && values.size() != 4) return false;

    pose.x = values[0];
    pose.y = values[1];
    pose.theta = values[2];

    has_timestamp = (values.size() == 4);
    timestamp_ms = has_timestamp ? static_cast<int64_t>(values[3]) : now_ms();
    return true;
}

static ParsedObjects parse_objects(const std::string& msg) {
    std::vector<Object2D> objects;

    ParsedObjects parsed;
    parsed.timestamp_ms = now_ms();
    parsed.has_timestamp = false;

    std::string payload = msg;
    size_t sep = msg.find('|');
    if (sep != std::string::npos) {
        try {
            parsed.timestamp_ms = std::stoll(msg.substr(0, sep));
            parsed.has_timestamp = true;
        } catch (...) {
            parsed.has_timestamp = false;
        }
        payload = msg.substr(sep + 1);
    }

    std::stringstream ss(payload);
    std::string item;

    while (std::getline(ss, item, ';')) {
        if (item.empty()) continue;

        std::stringstream pair_stream(item);
        std::string x_str, y_str;

        if (std::getline(pair_stream, x_str, ',') &&
            std::getline(pair_stream, y_str, ',')) {
            try {
                Object2D obj;
                obj.x = std::stod(x_str);
                obj.y = std::stod(y_str);
                objects.push_back(obj);
            } catch (...) {
                continue;
            }
        }
    }

    parsed.objects = std::move(objects);
    return parsed;
}

static cv::Point world_to_image(double x, double y,
                                int img_size,
                                double width_m,
                                double height_m) {

    double scale_x = img_size / width_m;
    double scale_y = img_size / height_m;

    int px = static_cast<int>(x * scale_x);
    int py = static_cast<int>(img_size - y * scale_y);

    if (px < 0) px = 0;
    if (py < 0) py = 0;
    if (px >= img_size) px = img_size - 1;
    if (py >= img_size) py = img_size - 1;

    return cv::Point(px, py);
}

int main(int argc, char** argv) {

    bool force_gui = false;
    bool force_headless = false;
    for (int i = 1; i < argc; ++i) {
        std::string arg(argv[i]);
        if (arg == "--gui") {
            force_gui = true;
        } else if (arg == "--headless") {
            force_headless = true;
        } else if (arg == "--help" || arg == "-h") {
            std::cout << "Usage: " << argv[0] << " [--gui|--headless]" << std::endl;
            std::cout << "  --gui       Force GUI window rendering" << std::endl;
            std::cout << "  --headless  Disable GUI window rendering" << std::endl;
            return 0;
        }
    }

    if (force_gui && force_headless) {
        std::cerr << "Cannot use both --gui and --headless together." << std::endl;
        return 1;
    }

    // ---- Qt/OpenCV display compatibility settings ----
    // Avoid hard-forcing xcb; let users override externally when needed.
    setenv("QT_XCB_GL_INTEGRATION", "none", 0);
    setenv("LIBGL_ALWAYS_SOFTWARE", "1", 0);
    setenv("QT_XCB_NO_XRANDR", "1", 0);
    setenv("QT_XCB_NO_XI2", "1", 0);

    const char* display_env = std::getenv("DISPLAY");
    bool has_display = (display_env != nullptr && std::strlen(display_env) > 0);
    bool headless = !has_display;
    if (force_headless) headless = true;
    if (force_gui) headless = false;

    std::cout << "DISPLAY=" << (has_display ? display_env : "<unset>") << std::endl;
    std::cout << "Render mode: " << (headless ? "headless" : "GUI") << std::endl;

    if (!headless && !has_display) {
        std::cerr << "GUI mode requested but DISPLAY is not set." << std::endl;
        std::cerr << "Set DISPLAY (example: export DISPLAY=:0) or run with --headless."
                  << std::endl;
        return 1;
    }

    if (headless) {
        setenv("QT_QPA_PLATFORM", "offscreen", 0);
        std::cout << "No DISPLAY detected. Running in headless mode (no GUI window)."
                  << std::endl;
    }
    // ---------------------------------------------------

    const int robot_port = 8888;
    const int objects_port = 8890;

    const double table_width_m = 3.0;
    const double table_height_m = 2.0;
    const double object_frame_yaw_offset_rad = -M_PI / 2.0;

    int robot_sock = make_udp_socket(robot_port);
    int objects_sock = make_udp_socket(objects_port);

    if (robot_sock < 0 || objects_sock < 0) {
        std::cerr << "Failed to open UDP sockets" << std::endl;
        return 1;
    }

    Pose2D robot_pose{0.0, 0.0, 0.0};
    int64_t robot_pose_ts_ms = now_ms();
    std::deque<TimedPose> pose_history;
    pose_history.push_back(TimedPose{robot_pose, robot_pose_ts_ms});

    std::vector<Object2D> objects_robot_frame;
    int64_t objects_ts_ms = now_ms();
    int pose_packets = 0;
    int object_packets = 0;

    const int img_size = 800;

    if (!headless) {
        try {
            cv::namedWindow("Global Frame", cv::WINDOW_NORMAL);
        } catch (const cv::Exception& e) {
            std::cerr << "Failed to create GUI window: " << e.what() << std::endl;
            std::cerr << "Falling back to headless mode." << std::endl;
            headless = true;
        }
    }

    std::cout << "Listening for robot pose on UDP port "
              << robot_port << std::endl;
    std::cout << "Listening for objects on UDP port "
              << objects_port << std::endl;
    std::cout << "Objects message format: x,y;x,y;... (robot frame, meters)"
              << std::endl;

    while (true) {

        // ---- Receive robot pose ----
        char buf[256];
        sockaddr_in src;
        socklen_t srclen = sizeof(src);

        int n = recvfrom(robot_sock, buf, sizeof(buf) - 1,
                         0, reinterpret_cast<sockaddr*>(&src), &srclen);

        if (n > 0) {
            buf[n] = '\0';
            Pose2D pose;
            int64_t pose_t_ms = now_ms();
            bool has_pose_ts = false;
            if (parse_pose(buf, pose, pose_t_ms, has_pose_ts)) {
                robot_pose = pose;
                robot_pose_ts_ms = pose_t_ms;
                pose_history.push_back(TimedPose{robot_pose, robot_pose_ts_ms});
                while (pose_history.size() > 2 &&
                       (pose_history.back().timestamp_ms - pose_history.front().timestamp_ms) > 5000) {
                    pose_history.pop_front();
                }
                ++pose_packets;
            }
        }

        // ---- Receive objects ----
        char obj_buf[1024];

        n = recvfrom(objects_sock, obj_buf, sizeof(obj_buf) - 1,
                     0, reinterpret_cast<sockaddr*>(&src), &srclen);

        if (n > 0) {
            obj_buf[n] = '\0';
            ParsedObjects parsed = parse_objects(obj_buf);
            objects_robot_frame = std::move(parsed.objects);
            objects_ts_ms = parsed.timestamp_ms;
            ++object_packets;
        }

        int64_t used_pose_ts_ms = robot_pose_ts_ms;
        Pose2D sync_pose = robot_pose;
        if (!pose_history.empty()) {
            sync_pose = pose_at_time(pose_history, objects_ts_ms, used_pose_ts_ms);
        }

        // ---- Render frame ----
        cv::Mat img(img_size, img_size, CV_8UC3,
                    cv::Scalar(255, 255, 255));

        // Table border
        cv::rectangle(img,
            world_to_image(0, 0, img_size, table_width_m, table_height_m),
            world_to_image(table_width_m, table_height_m,
                           img_size, table_width_m, table_height_m),
            cv::Scalar(0, 0, 0), 2);

        // Robot
        cv::Point robot_pt =
            world_to_image(robot_pose.x, robot_pose.y,
                           img_size, table_width_m, table_height_m);

        cv::circle(img, robot_pt, 6,
                   cv::Scalar(0, 0, 255), -1);

        // Heading
        double hx = robot_pose.x + 0.15 * std::cos(robot_pose.theta);
        double hy = robot_pose.y + 0.15 * std::sin(robot_pose.theta);

        cv::line(img,
                 robot_pt,
                 world_to_image(hx, hy,
                                img_size, table_width_m, table_height_m),
                 cv::Scalar(0, 0, 255), 2);

        // Objects
        for (const auto& obj : objects_robot_frame) {

            const double obj_theta = sync_pose.theta + object_frame_yaw_offset_rad;

            double gx =
                sync_pose.x +
                std::cos(obj_theta) * obj.x -
                std::sin(obj_theta) * obj.y;

            double gy =
                sync_pose.y +
                std::sin(obj_theta) * obj.x +
                std::cos(obj_theta) * obj.y;

            cv::Point obj_pt =
                world_to_image(gx, gy,
                               img_size, table_width_m, table_height_m);

            cv::circle(img, obj_pt, 4,
                       cv::Scalar(0, 150, 0), -1);
        }

                long long sync_dt_ms = static_cast<long long>(used_pose_ts_ms - objects_ts_ms);
                std::string status = "pose_pkts=" + std::to_string(pose_packets) +
                     "  obj_pkts=" + std::to_string(object_packets) +
                     "  objs=" + std::to_string(objects_robot_frame.size()) +
                     "  sync_dt_ms=" + std::to_string(sync_dt_ms);
            cv::putText(img, status, cv::Point(10, 25),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6,
                    cv::Scalar(80, 80, 80), 1, cv::LINE_AA);

        if (!headless) {
            cv::imshow("Global Frame", img);

            int key = cv::waitKey(20) & 0xFF;
            if (key == 'q' || key == 27)
                break;
        } else {
            usleep(20 * 1000);
        }
    }

    close(robot_sock);
    close(objects_sock);

    return 0;
}
//build: g++ global_plotter.cpp -o global_plotter `pkg-config --cflags --libs opencv4`
//run: 