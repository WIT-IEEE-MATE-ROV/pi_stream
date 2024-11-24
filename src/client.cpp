#include <iostream>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <unistd.h>
#include <string.h>
#include <vector>
#include <stdlib.h>
#include <errno.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>

#include "udpclient.h"

// To Timestamp each frame
std::string getCurrentTimestamp()
{
    auto now = std::chrono::system_clock::now();
    time_t time = std::chrono::system_clock::to_time_t(now);
    std::cout << "Time: " << time << std::endl;
    // Format the time
    std::string time_string = ctime(&time);
    return time_string;
}

void streamVideoOverUDP(const std::string &ipAddress, int port, int cameraIndex, int width, int height)
{
    if (width <= 0 || height <= 0)
    {
        std::cerr << "Frame size too small\n";
    }

    // Open the camera
    cv::VideoCapture cap;
    int apiID = cv::CAP_V4L2;
    cap.open(cameraIndex, apiID);

    if (!cap.isOpened())
    {
        std::cerr << "Error: Could not open the camera!" << std::endl;
        return;
    }

    std::cout << "Resolution set before cat.get(): " << width << "x" << height << std::endl;

    // Set resloution
    cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    int width_t = (int)cap.get(cv::CAP_PROP_FRAME_WIDTH);
    int height_t = (int)cap.get(cv::CAP_PROP_FRAME_HEIGHT);

    std::cout << "Resolution set after cat.get(): " << width_t << "x" << height_t << std::endl;

    // init socket
    int udpSocket = socket(AF_INET, SOCK_DGRAM, 0);
    if (udpSocket < 0)
    {
        std::cerr << "Error: Could not create socket!" << std::endl;
        return;
    }

    // config UDP stuff
    sockaddr_in destAddr{};
    destAddr.sin_family = AF_INET;
    destAddr.sin_port = htons(port);
    if (inet_pton(AF_INET, ipAddress.c_str(), &destAddr.sin_addr) <= 0)
    {
        std::cerr << "Error: Invalid IP address!" << std::endl;
        close(udpSocket);
        return;
    }

    const int maxPacketSize = (width_t * height_t) + 1;
    std::vector<uchar> buffer;
    cv::Mat frame;

    while (true)
    {
        cap >> frame;
        if (frame.empty())
        {
            std::cerr << "Error: Frame capture failed!" << std::endl;
            break;
        }

        if (!cv::imencode(".jpg", frame, buffer))
        {
            std::cerr << "Error: Could not encode frame! " << strerror(errno) << std::endl;
            break;
        }

        // Make sure frame fits into UDP packet
        // TODO - Chunk data for larger resloutions.
        if (buffer.size() > maxPacketSize)
        {
            std::cerr << "Warning: Frame too large for a single UDP packet. Skipping... " << strerror(errno) << std::endl;
            continue;
        }

        // Send frame over UDP
        ssize_t sentBytes = sendto(udpSocket, buffer.data(), buffer.size(), 0,
                                   (sockaddr *)(&destAddr), sizeof(destAddr));
        std::cout << "Bytes send " << sentBytes << std::endl;
        if (sentBytes < 0)
        {
            std::cerr << "Error: Failed to send frame! " << strerror(errno) << std::endl;
            break;
        }

        // ESC to exit
        cv::imshow("Camera Feed", frame);
        if (cv::waitKey(1) == 27)
        { 
            break;
        }
    }
    cap.release();
    close(udpSocket);
    cv::destroyAllWindows();
}
