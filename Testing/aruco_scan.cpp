#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <vector>
#include <stdio.h>

int main()
{
    // Use V4L2 device exposed by libcamera-v4l2 (same sensor as qcam)
    cv::VideoCapture cap(0, cv::CAP_V4L2); // force V4L2 backend on /dev/video0
    if (!cap.isOpened()) {
        printf("Camera open failed (V4L2 /dev/video0)\n");
        return -1;
    }

    cv::Ptr<cv::aruco::Dictionary> dict =
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

    int frameCount = 0;

    while (1) {
        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) break;

        frameCount++;

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;

        cv::aruco::detectMarkers(frame, dict, corners, ids);

        if (!ids.empty()) {
            for (size_t i = 0; i < ids.size(); i++) {
                printf("Detected ArUco ID: %d\n", ids[i]);
            }
            cv::aruco::drawDetectedMarkers(frame, corners, ids);
        } else if (frameCount % 60 == 0) {
            // About once a second at ~60 FPS, indicate we are still running
            printf("No markers detected yet (frame %d)\n", frameCount);
        }

        cv::imshow("ArUco Scanner", frame);
        if (cv::waitKey(1) == 27) break; // ESC to quit
    }

    return 0;
}
