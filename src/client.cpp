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
#include <libcamera/libcamera.h>
#include <libcamera/camera_manager.h>

extern "C"
{
#include "libavformat/avformat.h"
#include "libavcodec/avcodec.h"
#include "libavutil/opt.h"
#include "libavutil/hwcontext.h"
}

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
    int apiID = cv::CAP_ANY;
    cap.open(cameraIndex, apiID);

    if (!cap.isOpened())
    {
        std::cerr << "Error: Could not open the camera!" << std::endl;
        return;
    }

    std::cout << "Resolution set before cap.get(): " << width << "x" << height << std::endl;

    // Set resloution
    //cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
    //cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    //int width_cap = (int)cap.get(cv::CAP_PROP_FRAME_WIDTH);
    //int height_cap = (int)cap.get(cv::CAP_PROP_FRAME_HEIGHT);

    //std::cout << "Resolution set after cap.get(): " << width_cap << "x" << height_cap << std::endl;

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

//    const int maxPacketSize = (width_cap * height_cap) + 1;
    const int maxPacketSize = 320*180 + 1;
    std::cout << "Max packet size = " << maxPacketSize << std::endl;
    std::vector<uchar> buffer;
    cv::Mat frame;
    std::string hellostr = "Wassup\n";
    cap >> frame;
    cv::imwrite("test4.jpg", frame);
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

	// cv::imwrite("test.jpg", frame);

        // Make sure frame fits into UDP packet
        // TODO - Chunk data for larger resloutions.
        if (buffer.size() > maxPacketSize)
        {
            std::cerr << "Warning: Frame too large for a single UDP packet. Skipping... " << strerror(errno) << std::endl;
            continue;
        }



        // Send frame over UDP
        ssize_t sentBytes = sendto(udpSocket, buffer.data(), buffer.size(), 0, (sockaddr *)(&destAddr), sizeof(destAddr));
	// ssize_t sentBytes = sendto(udpSocket, hellostr.c_str(), hellostr.size() + 1, 0, (sockaddr *)(&destAddr), sizeof(destAddr));
        std::cout << "Bytes sent " << sentBytes << std::endl;
        if (sentBytes < 0)
        {
            std::cerr << "Error: Failed to send frame! " << strerror(errno) << std::endl;
            break;
        }
    }
    cap.release();
    close(udpSocket);

}
