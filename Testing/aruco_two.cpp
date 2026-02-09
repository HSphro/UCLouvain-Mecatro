#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <vector>
#include <stdio.h>
#include <iostream>

int main()
{
    // Open the default camera using the V4L2 backend
    cv::VideoCapture cap(0, cv::CAP_V4L2);

    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open camera." << std::endl;
        return -1;
    }

    // --- STABILITY SETTINGS ---
    // 1. Force a modest resolution (640x480 is standard for ArUco)
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    
    // 2. Reduce FPS to 20. This prevents the "libcamerify" wrapper from 
    //    choking/freezing if the GUI is slow to draw.
    cap.set(cv::CAP_PROP_FPS, 20);

    // 3. Set Buffer Size to 1. This is crucial! It tells the driver 
    //    "Don't queue up old frames, just give me the newest one."
    cap.set(cv::CAP_PROP_BUFFERSIZE, 1);
    // ---------------------------

    cv::Ptr<cv::aruco::Dictionary> dict =
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();

    std::cout << "Scanner started. Press ESC to quit..." << std::endl;

    while (true) {
        cv::Mat frame;
        // Attempt to read a frame
        bool success = cap.read(frame);

        if (!success || frame.empty()) {
            // If we lose the stream, log it but don't crash immediately 
            // (The wrapper might recover in the next loop)
            std::cout << "Frame dropped/empty..." << std::endl;
            continue; 
        }

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;

        cv::aruco::detectMarkers(frame, dict, corners, ids, parameters);

        if (!ids.empty()) {
            cv::aruco::drawDetectedMarkers(frame, corners, ids);
            // Print ID of the first marker found to console
            std::cout << "Detected ID: " << ids[0] << std::endl;
        }

        cv::imshow("ArUco Scanner", frame);

        // Reduced wait time to 1ms to process the buffer faster
        char key = (char)cv::waitKey(1);
        if (key == 27) break; 
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}

//g++ -Wall -Wextra -pedantic aruco_two.cpp -o aruco_two `pkg-config --cflags --libs opencv4`
// libcamerify ./aruco_two on the real_VNC
//pushed_correctly