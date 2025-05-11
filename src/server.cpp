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

#define PRINTV(EXP) (std::cout << #EXP << " = " << EXP << '\n')

class Server
{
public:

Server(const std::string &dest_ip) {
    dest_ip_ = dest_ip;
    std::cout << "Sending frames to " << dest_ip_ << std::endl;
}


private:
    const uint16_t PORT = 12345;
    const uint32_t WIDTH = 640, HEIGHT = 480;
    const uint32_t MAX_PACKET_SIZE = WIDTH * HEIGHT + 1;
    const int32_t FPS = 30;
    const int32_t FPS_US = 1000000 / FPS;
    const uint32_t PRINT_INTERVAL = 100;

    uint32_t stride_ = 0;
    uint32_t img_count_ = 0;
    std::unique_ptr<FileSink> sink_;
    std::shared_ptr<libcamera::Camera> camera_;
    int udp_socket_ = 0;
    int64_t pts_ = 0;
    sockaddr_in dest_addr_{};
    const AVCodec *codec_;
    AVCodecContext *codec_context_;
    AVFrame *av_frame_;
    AVPacket *pkt_;
    std::string dest_ip_;

    int encodeImage(const Image &image)
    {
        if (img_count_ % PRINT_INTERVAL == 0) {
            std::cout << "Format = " << av_get_pix_fmt_name(static_cast<AVPixelFormat>(av_frame_->format)) 
                << "\ndim = " <<  codec_context_->width << 'x' << codec_context_->height 
                << "\nstride = " << stride_ 
                << "\ny_plane size = " << image.data(0).size()
                << "\nu_plane size = " << image.data(1).size()
                << "\nv_plane size = " << image.data(2).size()
                << "\nn planes = " << image.numPlanes()
                << std::endl;
        }
        
        // YUV420 Planar
        const uint8_t *y_plane = image.data(0).data();
        const uint8_t *u_plane = image.data(1).data(); 
        const uint8_t *v_plane = image.data(2).data();
        
        av_frame_->data[0] = const_cast<uint8_t *>(y_plane);
        av_frame_->data[1] = const_cast<uint8_t *>(u_plane);
        av_frame_->data[2] = const_cast<uint8_t *>(v_plane);
        av_frame_->data[3] = NULL;

        av_frame_->linesize[0] = stride_;
        av_frame_->linesize[1] = stride_ / 2;
        av_frame_->linesize[2] = stride_ / 2;
        av_frame_->linesize[3] = 0;

        av_frame_->pts = ++pts_;

        int ret;
        if ((ret = avcodec_send_frame(codec_context_, av_frame_)) != 0) {
            char errstr[64];
            av_strerror(ret, errstr, 64);
            std::cerr << "Error sending to encoder: " << errstr << std::endl;
            av_frame_free(&av_frame_);
            avcodec_free_context(&codec_context_);
            return ret;
        }

        if ((ret = avcodec_receive_packet(codec_context_, pkt_)) != 0) {
            char errstr[64];
            av_strerror(ret, errstr, 64);
            std::cerr << "Failed to recieve packet from encoder: " << errstr << std::endl;
            return ret;
        }

        return ret;
    }

    void requestCompleteCB(libcamera::Request *request)
    {
        bool print = img_count_ % PRINT_INTERVAL == 0;
        ++img_count_;
        
        libcamera::FrameBuffer *frameBuffer = request->buffers().begin()->second;

        // mmap frame buffer to image plane buffers
        auto image = Image::fromFrameBuffer(frameBuffer, Image::MapMode::ReadOnly);

        auto encodeStart = std::chrono::high_resolution_clock::now();
        int ret = encodeImage(*image);
        auto encodeEnd = std::chrono::high_resolution_clock::now();

        if (print) {
            std::cout << "Encode time: " << std::chrono::duration_cast<std::chrono::milliseconds>(encodeEnd - encodeStart).count() << "ms\n"; 
        }

        // Send to UDP client
        if (ret == 0) {
            auto sendStart = std::chrono::high_resolution_clock::now();
            ssize_t sentBytes = sendto(udp_socket_, pkt_->data, pkt_->size, 0, (sockaddr *)(&dest_addr_), sizeof(dest_addr_));
            auto sendEnd = std::chrono::high_resolution_clock::now();
            if (print) {
                std::cout <<  "Send time: " << std::chrono::duration_cast<std::chrono::milliseconds>(sendEnd - sendStart).count() << "ms\n";
                std::cout << "Bytes sent " << sentBytes << "\n\n";
            }

            av_packet_unref(pkt_);

            if (sentBytes < 0)
            {
                std::cerr << "Error: Failed to send frame: " << strerror(errno) << std::endl;
            }
        } else {
            char errstr[64];
            av_strerror(ret, errstr, 64);
            std::cerr << "\nError occured encoding the av packet: " << errstr << std::endl;
        }

        request->reuse(libcamera::Request::ReuseBuffers);
        camera_->queueRequest(request);
    }

    void sinkRelease(libcamera::Request *request)
    {
        request->reuse(libcamera::Request::ReuseBuffers);
        camera_->queueRequest(request);
    }

public:
    ~Server() {
        if (codec_context_ != NULL) {
            avcodec_free_context(&codec_context_);
        }
    }

    int run()
    {
        // codec_ = avcodec_find_encoder(AV_CODEC_ID_H264);     // ~20ms 640x480
        codec_ = avcodec_find_encoder_by_name("h264_v4l2m2m");  // ~4ms 640x480
        if (!codec_) {
            std::cerr << "H.264 codec not found" << std::endl;
            return 1;
        }

        codec_context_ = avcodec_alloc_context3(codec_);
        if (!codec_context_) {
            std::cerr << "Couldn't allocate codec context" << std::endl;
            return 1;
        }

        codec_context_->bit_rate = 1000000;
        codec_context_->width = WIDTH;
        codec_context_->height = HEIGHT;
        codec_context_->time_base = {1, FPS};
        codec_context_->framerate = {FPS, 1};
        codec_context_->gop_size = FPS;
        // codec_context_->max_b_frames = 2;
        // codec_context_->refs = 3;
        codec_context_->pix_fmt = AV_PIX_FMT_YUV420P;
        
        if (av_opt_set(codec_context_->priv_data, "preset", "ultrafast", 0)) {
            std::cout << "Failed to set ultrafast option" << std::endl;
        }
        if (av_opt_set(codec_context_->priv_data, "crf", "18", 0)) {
            std::cout << "Failed to set crf option" << std::endl;
        }
        if (av_opt_set(codec_context_->priv_data, "tune", "zerolatency", 0)) {
            std::cout << "Failed to set tune option" << std::endl;
        }
        if (av_opt_set(codec_context_->priv_data, "profile", "baseline", 0)) {
            std::cout << "Failed to set profile option" << std::endl;
        }
        if (av_opt_set(codec_context_->priv_data, "level", "3.0", 0)) {
            std::cout << "Failed to set level option" << std::endl;
        }
        if (av_opt_set(codec_context_->priv_data, "repeat_spspps", "1", 0)) {
            std::cout << "Failed to set repeat_spspps option" << std::endl;
        }

        int ret; 
        if ((ret = avcodec_open2(codec_context_, codec_, nullptr)) != 0) {
            char errstr[64];
            av_strerror(ret, errstr, 64);
            std::cerr << "Couldn't open codec: " << errstr << std::endl;
            avcodec_free_context(&codec_context_);
            return 1;
        }

        av_frame_ = av_frame_alloc();
        av_frame_->format = codec_context_->pix_fmt;
        av_frame_->width = codec_context_->width;
        av_frame_->height = codec_context_->height;
        if ((ret = av_frame_get_buffer(av_frame_, 32)) != 0) {
            char errstr[64];
            av_strerror(ret, errstr, 64);
            std::cerr << "Couldn't get frame buffer: " << errstr << std::endl;
        }

        pkt_ = av_packet_alloc();

        udp_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (udp_socket_ < 0) {
            std::cerr << "Error: Could not create socket: " << strerror(errno) << std::endl;
            return 1;
        }
        
        dest_addr_.sin_family = AF_INET;
        dest_addr_.sin_port = htons(PORT);
        if (inet_pton(AF_INET, dest_ip_.c_str(), &dest_addr_.sin_addr) <= 0) {
            std::cerr << "Error: Invalid IP address!" << std::endl;
            close(udp_socket_);
            return 1;
        }
    
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

        camera_ = cm->get(camID);
        if (!camera_)
        {
            std::cerr << "Couldn't get camera" << std::endl;
            cm->stop();
            return 1;
        }

        std::cout << "Trying " << camID << std::endl;

        if (camera_->acquire())
        {
            std::cerr << "Couldn't acquire camera" << std::endl;
            cm->stop();
            return 1;
        }

        std::cout << "Acquired " << camID << std::endl;

        std::unique_ptr<libcamera::CameraConfiguration> config = camera_->generateConfiguration({libcamera::StreamRole::Viewfinder});

        config->orientation = libcamera::Orientation::Rotate180;

        for (libcamera::StreamConfiguration &stream_config : *config) {
            stream_config.pixelFormat = libcamera::formats::YUV420;
        }

        libcamera::logSetLevel("RPI", "WARN");
        libcamera::logSetLevel("Camera", "WARN");

        switch (config->validate()) {
            case libcamera::CameraConfiguration::Valid:
                std::cout << "Configuration valid" << std::endl;
                break;
        
            case libcamera::CameraConfiguration::Adjusted:
                std::cout << "Camera configuration adjusted" << std::endl;
                break;
        
            case libcamera::CameraConfiguration::Invalid:
                std::cout << "Camera configuration invalid" << std::endl;
                return 1;
        }

        // TODO: Try color-balancing configurations when underwater
        auto fd_ctrl = camera_->controls().find(&libcamera::controls::FrameDurationLimits);
        double framerate = 1.0e6 / fd_ctrl->second.min().get<int64_t>();
        std::cout << "framerate:(" << framerate << ") \n"
            << "bitdepth:(" << config->sensorConfig->bitDepth << ") \n\n";

        stride_ = config->at(0).stride;

        for (libcamera::StreamConfiguration &cfg : *config) {
            cfg.size.width = WIDTH;
            cfg.size.height = HEIGHT;
            std::cout << "Sensor Mode: size:(" << cfg.size.width << "x" << cfg.size.height << ") " 
                << "pixel format:(" << cfg.pixelFormat.toString() << ") " 
                << "stride:(" << cfg.stride << ") "
                << "bufferCount:(" << cfg.bufferCount << ") "
                << "frameSize:(" << cfg.frameSize << ") "
                << std::endl;
        }

        if (camera_->configure(config.get())) {
            std::cout << "Failed to configure camera" << std::endl;
            return 1;
        }

        std::map<const libcamera::Stream *, std::string> streamNames_;

        for (int i = 0; i < config->size(); i++)
        {
            libcamera::StreamConfiguration &cfg = config->at(i);
            streamNames_[cfg.stream()] = "cam" + std::to_string(camNum) + "-stream" + std::to_string(i);
        }

        sink_ = std::make_unique<FileSink>(camera_.get(), streamNames_);
        sink_->setFilePattern("captures/server.cpp_#.ppm");
        ret = sink_->configure(*config);
        if (ret < 0)
        {
            std::cout << "Failed to configure frame sink: " << ret << std::endl;
            return ret;
        }

        sink_->requestProcessed.connect(this, &Server::sinkRelease);

        auto allocator = std::make_unique<libcamera::FrameBufferAllocator>(camera_);

        if (config->size() > 1)
        {
            std::cerr << "More than one config" << std::endl;
            return 1;
        }

        libcamera::StreamConfiguration &cfg = config->at(0);

        ret = allocator->allocate(cfg.stream());
        if (ret < 0)
        {
            std::cerr << "Can't allocate buffers: " << ret << std::endl;
            return -ENOMEM;
        }

        unsigned int allocated = allocator->buffers(cfg.stream()).size();

        std::vector<std::unique_ptr<libcamera::Request>> requests;

        for (auto& buffer : allocator->buffers(cfg.stream()))
        {
            std::unique_ptr<libcamera::Request> request = camera_->createRequest();
            if (!request)
            {
                std::cerr << "Can't create request" << std::endl;
                return -ENOMEM;
            }

            ret = request->addBuffer(cfg.stream(), buffer.get());
            if (ret < 0) {
                std::cerr << "Can't set buffer for request" << std::endl;
                return ret;
            }

            sink_->mapBuffer(buffer.get());

            requests.push_back(std::move(request));
        }

        libcamera::ControlList controls;
        controls.set(libcamera::controls::FrameDurationLimits, libcamera::Span<const int64_t, 2>({FPS_US, FPS_US}));

        camera_->requestCompleted.connect(this, &Server::requestCompleteCB);

        ret = camera_->start(&controls);
        if (ret < 0)
        {
            std::cerr << "Failed to start camera: " << ret << std::endl;
            cm->stop();
            return 1;
        } 
        else 
        {
            std::cout << "Started camera" << std::endl;
        }

        for (auto &request : requests)
        {
            std::cout << "Queueing request: " << request->toString() << std::endl;
            ret = camera_->queueRequest(request.get());
        }

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

        std::cout << "Stopping" << std::endl;

        cm->stop();

        return 0;
    }
};

// TODO: Command line arguments
int main(const int argc, const char *argv[])
{
    if (argc <= 1) {
        std::cerr << "Requires IP argument: '" << argv[0] << " --ip=[IP ADDRESS]'\n"
            << "Good day sir." << std::endl;
        return 1;
    }

    std::string arg1 = argv[1];
    // TODO: finish Command line arguments
    size_t split_pos = arg1.find('=');
    if (split_pos == std::string::npos) {
        std::cerr << "Invalid argument input: '" << argv[0] << " --ip=[IP ADDRESS]'\n";
        return 1;
    }

    std::string ip(arg1.begin() + split_pos + 1, arg1.end());
    
    return Server(ip).run();
}