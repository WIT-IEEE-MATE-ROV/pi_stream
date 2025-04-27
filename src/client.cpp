#include <iostream>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <unistd.h>
#include <cstring>
#include <vector>
#include <cstdlib>
#include <cerrno>
#include <optional>
#include <csignal>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

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
#include "apps/cam/camera_session.h"
#include "apps/common/event_loop.h"


using namespace libcamera;


void Client::captureDone() {
    std::cout << "Capture done" << std::endl;
    done = true;
    evloop.exit();
}

void Client::run() {
    libcamera::CameraManager cm;
    if (cm.start()) {
        printf("Couldn't start camera manager\n");
        return;
    }

    CameraSession::CameraSessionSettings settings;
    settings.roles.push_back(libcamera::StreamRole::Viewfinder);
    settings.captureLimit = 1;
    settings.orientation = libcamera::Orientation::Rotate180;
    settings.outFile = "stream_test.ppm";
    settings.pixelFormat = libcamera::PixelFormat::fromString("BGR888");


    CameraSession session(&cm, "1", 1, settings);
    if (!session.isValid()) {
        std::cout << "Session invalid" << std::endl;
        return;
    }

    std::cout << "Session valid\n";
    std::cout << "Using camera " << session.camera()->id() << " as cam" << 1 << std::endl;

    session.captureDone.connect(this, &Client::captureDone);

    if (session.start()) {
        std::cout << "Failed to start camera session" << std::endl;
        cm.stop();
        return;
    }
    std::cout << "Session started" << std::endl;
    
    evloop.exec();

    std::cout << "Stopping" << std::endl;
    session.stop();
    cm.stop();
}

void Client::signalHandler(int signal) {
    std::cout << "Signal Exiting" << std::endl;
    done = true;
    evloop.exit();
}

void Client::streamVideoOverUDP(const std::string &ipAddress, int port, int cameraIndex, int width, int height)
{
    // struct sigaction sa{};
    // sa.sa_handler = &Client::signalHandler;
    // sigaction(SIGINT, &sa, nullptr);
    // run();
    // std::cout << "Exiting..." << std::endl;
    // return;

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
