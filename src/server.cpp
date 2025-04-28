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
#include "libavutil/opt.h"
#include "libavutil/imgutils.h"
}

#include "udpclient.h"
#include "apps/cam/camera_session.h"
#include "apps/cam/file_sink.h"
#include "apps/common/event_loop.h"
#include "apps/common/ppm_writer.h"
#include "apps/common/image.h"

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
        unsigned int mode_depth = fmt.find("8") != std::string::npos ? 8 : fmt.find("10") != std::string::npos ? 10
                                                                       : fmt.find("12") != std::string::npos   ? 12
                                                                       : fmt.find("14") != std::string::npos   ? 14
                                                                                                               : 16;
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

class Server
{

private:
    const uint16_t PORT = 12345;
    const std::string IP_ADDRESS = "10.0.0.135";
    const uint32_t WIDTH = 320, HEIGHT = 180;
    const uint32_t MAX_PACKET_SIZE = 320 * 180 + 1;
    const int32_t FPS = 20;
    const int32_t FPS_US = 1000000 / FPS;

    uint32_t stride_ = 0;
    uint32_t img_count = 0;
    std::unique_ptr<FileSink> sink;
    std::shared_ptr<libcamera::Camera> camera;
    std::vector<uchar> send_buffer_;
    std::map<const libcamera::Stream *, std::string> streamNames_;
    int udp_socket_ = 0;
    sockaddr_in dest_addr_{};
    const AVCodec *codec_;
    AVCodecContext *codec_context_;
    AVFrame *av_frame;
    

    void requestCompleteCB(libcamera::Request *request)
    {
        ++img_count;
        std::cout << "Completed request: " << request->toString() << std::endl;

        // bool success = sink->processRequest(request);
        bool success = true;

        cv::Mat frame(HEIGHT, WIDTH, CV_8UC3);
        libcamera::FrameBuffer *frameBuffer;
        for (auto &[stream, buffer] : request->buffers()) {
            frameBuffer = buffer;
            break;
        }

        // Copy image data from libcamera Image to cv Mat
        auto image = Image::fromFrameBuffer(frameBuffer, Image::MapMode::ReadOnly);
        const char *row = reinterpret_cast<const char *>(image->data(0).data());

        uint32_t row_length = WIDTH * 3;
        
        uint32_t frame_idx = 0;
        for (int32_t y = 0; y < HEIGHT; y++, row += stride_) {
            for (int32_t x = 0; x < row_length; x += 3, frame_idx += 3) {
                // BGR -> RGB
                frame.data[frame_idx] = row[x + 2];
                frame.data[frame_idx + 1] = row[x + 1];
                frame.data[frame_idx + 2] = row[x]; 
            }
        }


        cv::Mat yuv;
        cv::cvtColor(frame, yuv, cv::COLOR_BGR2YUV_I420);

        int y_size = codec_context_->width * codec_context_->height;
        memcpy(av_frame->data[0], yuv.data, y_size);
        memcpy(av_frame->data[1], yuv.data + y_size, y_size / 4);
        memcpy(av_frame->data[2], yuv.data + y_size + y_size / 4, y_size / 4);

        av_frame->pts = 0;

        if (avcodec_send_frame(codec_context_, av_frame) < 0) {
            std::cerr << "Error sending frame for encoding" << std::endl;
            av_frame_free(&av_frame);
            avcodec_free_context(&codec_context_);
            return;
        }

        AVPacket *pkt = av_packet_alloc();
        int ret = avcodec_receive_packet(codec_context_, pkt);
        if (ret == 0) {
            // TODO: Manipulate packet
            ssize_t sentBytes = sendto(udp_socket_, pkt->data, pkt->size, 0, (sockaddr *)(&dest_addr_), sizeof(dest_addr_));
            std::cout << "Bytes sent " << sentBytes << std::endl;
            if (sentBytes < 0)
            {
                std::cerr << "Error: Failed to send frame! " << strerror(errno) << std::endl;
            }
        }

        // char filename[100];
        // sprintf(filename, "captures/testframe_%u.jpg", img_count);
        // cv::imwrite(filename, frame);
        
        // if (!cv::imencode(".jpg", frame, send_buffer_))
        // {
        //     std::cerr << "Error: Could not encode frame! " << strerror(errno) << std::endl;
        // }

        // if (send_buffer_.size() > MAX_PACKET_SIZE) {
        //     std::cerr << "Warning: Frame too large for a single UDP packet. Skipping... " << strerror(errno) << std::endl;
        // }

        // ssize_t sentBytes = sendto(udp_socket_, send_buffer_.data(), send_buffer_.size(), 0, (sockaddr *)(&dest_addr_), sizeof(dest_addr_));
        // std::cout << "Bytes sent " << sentBytes << std::endl;
        // if (sentBytes < 0)
        // {
        //     std::cerr << "Error: Failed to send frame! " << strerror(errno) << std::endl;
        // }

        if (!success)
        {
            // requeue = false;
        } else {
            request->reuse(libcamera::Request::ReuseBuffers);
            camera->queueRequest(request);
        }
    }

    void sinkRelease(libcamera::Request *request)
    {
        request->reuse(libcamera::Request::ReuseBuffers);
        camera->queueRequest(request);
    }

public:
    int run()
    {
        codec_ = avcodec_find_encoder(AV_CODEC_ID_H264);
        if (!codec_) {
            std::cerr << "H.264 codec not found" << std::endl;
            return 1;
        }

        codec_context_ = avcodec_alloc_context3(codec_);
        if (!codec_context_) {
            std::cerr << "Couldn't allocate codec context" << std::endl;
            return 1;
        }

        codec_context_->bit_rate = 400000;
        codec_context_->width = WIDTH;
        codec_context_->height = HEIGHT;
        codec_context_->time_base = {1, FPS};
        codec_context_->framerate = {FPS, 1};
        codec_context_->gop_size = 10;
        codec_context_->max_b_frames = 1;
        codec_context_->pix_fmt = AV_PIX_FMT_YUV420P;

        if (avcodec_open2(codec_context_, codec_, nullptr) < 0) {
            std::cerr << "Couldn't open codec" << std::endl;
            avcodec_free_context(&codec_context_);
            return 1;
        }

        AVFrame *av_frame = av_frame_alloc();
        av_frame->format = codec_context_->pix_fmt;
        av_frame->width = codec_context_->width;
        av_frame->height = codec_context_->height;
        av_frame_get_buffer(av_frame, 32);


        udp_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (udp_socket_ < 0) {
            std::cerr << "Error: Could not create socket!" << std::endl;
            return 1;
        }
        
        dest_addr_.sin_family = AF_INET;
        dest_addr_.sin_port = htons(PORT);
        if (inet_pton(AF_INET, IP_ADDRESS.c_str(), &dest_addr_.sin_addr) <= 0) {
            std::cerr << "Error: Invalid IP address!" << std::endl;
            close(udp_socket_);
            return 1;
        }
    
        int ret;

        std::map<libcamera::FrameBuffer *, std::unique_ptr<Image>> mappedBuffers;

        auto cm = std::make_unique<libcamera::CameraManager>();
        if (cm->start())
        {
            std::cerr << "Failed to start camera manager" << std::endl;
            return 1;
        }

        auto cameras = cm->cameras();

        if (cameras.size() == 0)
        {
            std::cerr << "No cameras available" << std::endl;
            cm->stop();
            return 1;
        }

        std::cout << "Cameras available:\n";
        for (const auto &cam : cameras)
        {
            std::cout << cam->id() << '\n';
        }
        std::cout << std::endl;

        const int camNum = 0;
        const std::string &camID = cameras[camNum]->id();

        this->camera = cm->get(camID);
        if (!camera)
        {
            std::cerr << "Couldn't get camera" << std::endl;
            cm->stop();
            return 1;
        }

        std::cout << "Trying " << camID << std::endl;

        if (camera->acquire())
        {
            std::cerr << "Couldn't acquire camera" << std::endl;
            cm->stop();
            return 1;
        }

        std::cout << "Acquired" << std::endl;

        std::unique_ptr<libcamera::CameraConfiguration> config = camera->generateConfiguration({libcamera::StreamRole::Viewfinder});
        const libcamera::StreamFormats &formats = config->at(0).formats();

        config->orientation = libcamera::Orientation::Rotate180;
        for (int i = 0; i < config->size(); i++) {
            auto &cfg = config->at(i);
            cfg.pixelFormat = libcamera::PixelFormat::fromString("BGR888");
        }

        libcamera::logSetLevel("RPI", "ERROR");
        libcamera::logSetLevel("Camera", "ERROR");

        switch (config->validate()) {
            case libcamera::CameraConfiguration::Valid:
                std::cout << "Configuration valid" << std::endl;
                break;
        
            case libcamera::CameraConfiguration::Adjusted:
                // if (strictFormats) {
                //     std::cout << "Adjusting camera configuration disallowed by --strict-formats argument"
                //           << std::endl;
                //     return;
                // }
                std::cout << "Camera configuration adjusted" << std::endl;
                break;
        
            case libcamera::CameraConfiguration::Invalid:
                std::cout << "Camera configuration invalid" << std::endl;
                return 1;
        }

        
        for (int i = 0; i < config->size(); i++) {
            auto fd_ctrl = camera->controls().find(&libcamera::controls::FrameDurationLimits);
            double framerate = 1.0e6 / fd_ctrl->second.min().get<int64_t>();

            libcamera::StreamConfiguration &cfg = config->at(i);
            cfg.size.width = WIDTH;
            cfg.size.height = HEIGHT;
            stride_ = cfg.stride;
            std::cout << "Sensor Mode: size:(" << cfg.size.width << "x" << cfg.size.height << ") " 
            << "pixel format:(" << cfg.pixelFormat.toString() << ") " 
            << "bitdepth:(" << config->sensorConfig->bitDepth << ") " 
            << "stride:(" << cfg.stride << ") "
            << "bufferCount:(" << cfg.bufferCount << ") "
            << "frameSize:(" << cfg.frameSize << ") "
            << "framerate:(" << framerate << ") "
            << "stride:(" << cfg.stride << ")"
            << std::endl;
        }

        camera->configure(config.get());

        std::map<const libcamera::Stream *, std::string> streamNames_;

        for (int i = 0; i < config->size(); i++)
        {
            libcamera::StreamConfiguration &cfg = config->at(i);
            streamNames_[cfg.stream()] = "cam" + std::to_string(camNum);
            +"-stream" + std::to_string(i);
        }

        sink = std::make_unique<FileSink>(camera.get(), streamNames_);
        sink->setFilePattern("captures/server.cpp_#.ppm");
        ret = sink->configure(*config);
        if (ret < 0)
        {
            std::cout << "Failed to configure frame sink"
                      << std::endl;
            return ret;
        }

        sink->requestProcessed.connect(this, &Server::sinkRelease);

        auto allocator = std::make_unique<libcamera::FrameBufferAllocator>(camera);

        libcamera::StreamConfiguration cfg = config->at(0);
        std::cout << "Allocating buffers\nConfig length = " << config->size() << std::endl;

        if (config->size() > 1)
        {
            std::cerr << "More than one config" << std::endl;
            return 1;
        }

        unsigned int nbuffers = UINT_MAX;

        ret = allocator->allocate(cfg.stream());
        if (ret < 0)
        {
            std::cerr << "Can't allocate buffers" << std::endl;
            return -ENOMEM;
        }

        unsigned int allocated = allocator->buffers(cfg.stream()).size();
        nbuffers = std::min(nbuffers, allocated);

        std::vector<std::unique_ptr<libcamera::Request>> requests;

        std::cout << "nbuffers: " << nbuffers << std::endl; 
        for (unsigned int i = 0; i < nbuffers; i++)
        {
            std::unique_ptr<libcamera::Request> request = camera->createRequest();
            if (!request)
            {
                std::cerr << "Can't create request" << std::endl;
                return -ENOMEM;
            }

            libcamera::Stream *stream = cfg.stream();
            const std::vector<std::unique_ptr<libcamera::FrameBuffer>> &buffers = allocator->buffers(stream);
            const std::unique_ptr<libcamera::FrameBuffer> &buffer = buffers[i];

            ret = request->addBuffer(stream, buffer.get());
            if (ret < 0)
            {
                std::cerr << "Can't set buffer for request"
                          << std::endl;
                return ret;
            }

            // if (sink_)
            //     sink_->mapBuffer(buffer.get());
            sink->mapBuffer(buffer.get());

            requests.push_back(std::move(request));
        }

        libcamera::ControlList controls;
        controls.set(libcamera::controls::FrameDurationLimits, libcamera::Span<const int64_t, 2>({FPS_US, FPS_US}));

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
        camera->requestCompleted.connect(this, &Server::requestCompleteCB);

        if (camera->start(&controls))
        {
            std::cerr << "Failed to start camera" << std::endl;
            cm->stop();
            return 1;
        }
        else
        {
            std::cout << "Started camera" << std::endl;
        }

        for (auto &request : requests)
        {
            std::cout << "Queueing request" << std::endl;
            ret = camera->queueRequest(request.get());
        }

        // camera->

        auto start = std::chrono::high_resolution_clock::now();
        std::chrono::time_point now = start;
        while (true || std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() < 5000)
        {
            now = std::chrono::high_resolution_clock::now();
        }

        for (int i = 0; i < requests.size(); i++)
        {
            auto &req = requests[i];
            if (req->status() == libcamera::Request::RequestComplete)
            {
                // PPMWriter::write("server.cpp.ppm", cfg, );
            }
            else
            {
                // std::cerr << "Request failed to complete" << std::endl;
            }
        }

        std::cout << "stopping" << std::endl;

        cm->stop();

        return 0;
    }
};

int main(void)
{
    Server().run();
    return 0;
}