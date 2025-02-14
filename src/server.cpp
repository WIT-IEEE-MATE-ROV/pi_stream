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

struct SensorMode
{
    SensorMode()
        : size({}), format({}), fps(0)
    {
    }
    SensorMode(libcamera::Size _size, libcamera::PixelFormat _format, double _fps)
        : size(_size), format(_format), fps(_fps)
    {
    }
    unsigned int depth() const
    {
        // This is a really ugly way of getting the bit depth of the format.
        // But apart from duplicating the massive bayer format table, there is
        // no other way to determine this.
        std::string fmt = format.toString();
        unsigned int mode_depth = fmt.find("8") != std::string::npos ? 8 :
                                  fmt.find("10") != std::string::npos ? 10 :
                                  fmt.find("12") != std::string::npos ? 12 :
                                  fmt.find("14") != std::string::npos ? 14 : 16;
        return mode_depth;
    }
    libcamera::Size size;
    libcamera::PixelFormat format;
    double fps;
    std::string ToString() const
    {
        std::stringstream ss;
        ss << format.toString() << "," << size.toString() << "/" << fps;
        return ss.str();
    }
};

void requestCompleteCB() {
    std::cout << "Completed request" << std::endl;
}

int main() {
    auto cm = std::make_unique<libcamera::CameraManager>();
    if (cm->start()) {
        std::cerr << "Failed to start camera manager" << std::endl;
        return 1;
    }

    auto cameras = cm->cameras();

    if (cameras.size() == 0) {
        std::cerr << "No cameras available" << std::endl;
        cm->stop();
        return 1;
    }

    std::cout << "Cameras available:\n";
    for (const auto &cam : cameras) {
        std::cout << cam->id() << '\n';
    }
    std::cout << std::endl;

    const int camNum = 0;
    const std::string &camID = cameras[camNum]->id();

    std::shared_ptr<libcamera::Camera> camera = cm->get(camID);
    if (!camera) {
        std::cerr << "Couldn't get camera" << std::endl;
        cm->stop();
        return 1;
    }

    std::cout << "Trying " << camID << std::endl;

    if (camera->acquire()) {
        std::cerr << "Couldn't acquire camera" << std::endl;
        cm->stop();
        return 1;
    }

    std::cout << "Acquired" << std::endl;

    std::unique_ptr<libcamera::CameraConfiguration> config = camera->generateConfiguration({ libcamera::StreamRole::Raw });
    const libcamera::StreamFormats &formats = config->at(0).formats();

    libcamera::logSetLevel("RPI", "ERROR");
    libcamera::logSetLevel("Camera", "ERROR");

    std::vector<SensorMode> sensor_modes;
    for (const auto &pxf : formats.pixelformats()) {
        for (const auto &size : formats.sizes(pxf)) {
            double framerate = 0;
            SensorMode sensorMode(size, pxf, 0);
            config->at(0).size = size;
            config->at(0).pixelFormat = pxf;
            config->sensorConfig = libcamera::SensorConfiguration();
            config->sensorConfig->outputSize = size;
            config->sensorConfig->bitDepth = sensorMode.depth();
            config->validate();
            camera->configure(config.get());
            auto fd_ctrl = camera->controls().find(&libcamera::controls::FrameDurationLimits);
            framerate = 1.0e6 / fd_ctrl->second.min().get<int64_t>();

            sensor_modes.emplace_back(size, pxf, framerate);
        }
    }

    libcamera::logSetLevel("RPI", "INFO");
    libcamera::logSetLevel("Camera", "INFO");

    // std::vector<libcamera::StreamRole> stream_roles = { libcamera::StreamRole::VideoRecording };

    // bool low_res = false;
    // bool no_raw = false;

    // int lores_index = 1;
    // if (!no_raw) {
    //     stream_roles.push_back(libcamera::StreamRole::Raw);
    // }
    // if (low_res) {
    //     stream_roles.push_back(libcamera::StreamRole::Viewfinder);
    // }

    // config = camera->generateConfiguration(stream_roles);

    if (camera->start()) {
        std::cerr << "Failed to start camera" << std::endl;
        cm->stop();
        return 1;
    }
    // c             amera->
    // camera->requestCompleted.connect(nullptr, &requestCompleteCB);



    cm->stop();
    
    
    return 0;
}