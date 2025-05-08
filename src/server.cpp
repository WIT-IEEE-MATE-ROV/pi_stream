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

private:
    const uint16_t PORT = 12345;
    const std::string IP_ADDRESS = "10.0.0.135";
    const uint32_t WIDTH = 640, HEIGHT = 480;
    const uint32_t MAX_PACKET_SIZE = WIDTH * HEIGHT + 1;
    const int32_t FPS = 30;
    const int32_t FPS_US = 1000000 / FPS;

    uint32_t stride_ = 0;
    uint32_t img_count = 0;
    std::unique_ptr<FileSink> sink;
    std::shared_ptr<libcamera::Camera> camera;
    std::vector<uchar> send_buffer_;
    std::map<const libcamera::Stream *, std::string> streamNames_;
    int udp_socket_ = 0;
    int64_t pts_ = 0;
    sockaddr_in dest_addr_{};
    const AVCodec *codec_;
    AVCodecContext *codec_context_;
    AVFrame *av_frame;
    AVPacket *pkt;
    cv::Mat frame_;
    cv::Mat yuv_;
    uint32_t adjustment = 0;

    // TODO: Measure times to see what is causing latency when dimensions are increased above 320x180
    // TODO: Try that but without h.264 encoding

    int encodeImage(libcamera::FrameBuffer *frameBuffer)
    {
        
        // Copy image data from libcamera Image to cv Mat
        auto image = Image::fromFrameBuffer(frameBuffer, Image::MapMode::ReadOnly);

        const char *row = reinterpret_cast<const char *>(image->data(0).data());

        const uint8_t *image_data = image->data(0).data(); // raw NV12 data

        int y_stride = stride_; // actual stride (might be wider than width)
        int height = codec_context_->height;
        int y_plane_size = image->data(0).size();
        int uv_plane_size = image->data(1).size();

        if (img_count % 100 == 0) {
        std::cout << "Format = " << av_get_pix_fmt_name(static_cast<AVPixelFormat>(av_frame->format)) 
            << " \ndim = " <<  codec_context_->width << 'x' << codec_context_->height 
            << " \nstride = " << stride_ 
            << " \ny_plane_size = " << frameBuffer->planes()[0].length 
            << " \nuv_plane_size " << frameBuffer->planes()[1].length
            << "\nn planes = " << frameBuffer->planes().size()
            << std::endl;
        }

        // YUV420
        const uint8_t *y_plane = image->data(0).data();
        const uint8_t *u_plane = image->data(1).data(); 
        const uint8_t *v_plane = image->data(2).data();
        
        // Set data pointers based on expected NV12 layout
        av_frame->data[0] = const_cast<uint8_t *>(y_plane);                   // Y plane
        av_frame->data[1] = const_cast<uint8_t *>(u_plane); // UV plane
        av_frame->data[2] = const_cast<uint8_t *>(v_plane);
        av_frame->data[3] = NULL;

        av_frame->linesize[0] = stride_;
        av_frame->linesize[1] = stride_/2;
        av_frame->linesize[2] = stride_/2;
        av_frame->linesize[3] = 0;


        // NV12
        // const uint8_t *y_plane = image->data(0).data();
        // const uint8_t *uv_plane = image->data(1).data(); // safer
        // PRINTV(frameBuffer->planes()[0].length);
        // PRINTV(frameBuffer->planes()[1].length);
        // // const uint8_t *v_plane = image->data(2).data();
        // // av_frame->
        // // Set data pointers based on expected NV12 layout
        // av_frame->data[0] = const_cast<uint8_t *>(y_plane);                   // Y plane
        // av_frame->data[1] = const_cast<uint8_t *>(uv_plane); // UV plane
        // av_frame->data[2] = NULL;

        // uint32_t aligned_width  = (WIDTH + 31) & ~31;
        // uint32_t aligned_height = (HEIGHT + 15) & ~15;
        // av_frame->linesize[0] = (frameBuffer->planes()[0].length / HEIGHT + 31) & ~31;
        // av_frame->linesize[1] = (frameBuffer->planes()[0].length / HEIGHT + 31) & ~31;
        // av_frame->linesize[2] = 0;
        // PRINTV(av_frame->linesize[0]);
        // PRINTV(av_frame->linesize[1]);


        av_frame->pts = pts_++;

        // std::cout << "av_frame->format = " << av_get_pix_fmt_name(static_cast<AVPixelFormat>(av_frame->format)) << '\n';
        int ret;
        if ((ret = avcodec_send_frame(codec_context_, av_frame)) < 0) {
            char errstr[200];
            av_strerror(ret, errstr, 200);
            std::cerr << "Error sending to encoder: " << errstr << std::endl;
            av_frame_free(&av_frame);
            avcodec_free_context(&codec_context_);
            return ret;
        }

        ret = avcodec_receive_packet(codec_context_, pkt);
        if (ret != 0) {
            char errstr[200];
            av_strerror(ret, errstr, 200);
            std::cerr << "Failed to recieve packet from encoder: " << errstr << std::endl;
        }

        return ret;
    }

    void requestCompleteCB(libcamera::Request *request)
    {
        ++img_count;
        // std::cout << "Completed request: " << request->toString() << std::endl;

        // bool success = sink->processRequest(request);
        bool success = true;

        cv::Mat frame(HEIGHT, WIDTH, CV_8UC3);
        libcamera::FrameBuffer *frameBuffer;
        for (auto &[stream, buffer] : request->buffers()) {
            frameBuffer = buffer;
            break;
        }

        auto encodeStart = std::chrono::high_resolution_clock::now();
        int ret = encodeImage(frameBuffer);
        auto encodeEnd = std::chrono::high_resolution_clock::now();

        std::cout << "Encode time: " << std::chrono::duration_cast<std::chrono::milliseconds>(encodeEnd - encodeStart).count() << "ms\t"; 

        // Send to UDP client
        if (ret == 0) {
            ssize_t sentBytes = sendto(udp_socket_, pkt->data, pkt->size, 0, (sockaddr *)(&dest_addr_), sizeof(dest_addr_));
            std::cout << "Bytes sent " << sentBytes << "\n\n"; // Should be less than 10k
            av_packet_unref(pkt);
            if (sentBytes < 0)
            {
                std::cerr << "Error: Failed to send frame! " << strerror(errno) << std::endl;
            }
        } else {
            char errstr[200];
            av_strerror(ret, errstr, 200);
            std::cerr << "Error occured recieving the av packet: " << errstr << std::endl;
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
        frame_ = cv::Mat(HEIGHT, WIDTH, CV_8UC3);

        // codec_ = avcodec_find_encoder(AV_CODEC_ID_H264);
        codec_ = avcodec_find_encoder_by_name("h264_v4l2m2m"); // 28ms encode
        if (!codec_) {
            std::cerr << "H.264 codec not found" << std::endl;
            return 1;
        }

        codec_context_ = avcodec_alloc_context3(codec_);
        if (!codec_context_) {
            std::cerr << "Couldn't allocate codec context" << std::endl;
            return 1;
        }

        // codec_context_->bit_rate = 400000;
        codec_context_->width = WIDTH;
        codec_context_->height = HEIGHT;
        codec_context_->time_base = {1, FPS};
        codec_context_->framerate = {FPS, 1};
        codec_context_->gop_size = FPS;
        // codec_context_->max_b_frames = 3;
        // codec_context_->refs = 3;
        codec_context_->pix_fmt = AV_PIX_FMT_YUV420P;
        // codec_context_->pix_fmt = AV_PIX_FMT_NV12;
        
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

        if (avcodec_open2(codec_context_, codec_, nullptr) < 0) {
            std::cerr << "Couldn't open codec" << std::endl;
            avcodec_free_context(&codec_context_);
            return 1;
        }

        av_frame = av_frame_alloc();
        av_frame->format = codec_context_->pix_fmt;
        av_frame->width = codec_context_->width;
        av_frame->height = codec_context_->height;
        av_frame_get_buffer(av_frame, 32);

        pkt = av_packet_alloc();


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
            // cfg.pixelFormat = libcamera::PixelFormat::fromString("BGR888");
            // cfg.pixelFormat = libcamera::formats::NV12;
            cfg.pixelFormat = libcamera::formats::YUV420;
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
            << std::endl;
        }

        camera->configure(config.get());

        std::map<const libcamera::Stream *, std::string> streamNames_;

        for (int i = 0; i < config->size(); i++)
        {
            libcamera::StreamConfiguration &cfg = config->at(i);
            streamNames_[cfg.stream()] = "cam" + std::to_string(camNum) + "-stream" + std::to_string(i);
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
            std::cout << "Queueing request: " << request->toString() << std::endl;
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