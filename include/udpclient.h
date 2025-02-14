#pragma once

#include <cstring>
#include "apps/common/event_loop.h"

class Client {
public:
    bool done = false;
    EventLoop evloop;

    void captureDone();
    void streamVideoOverUDP(const std::string &ipAddress, int port, int cameraIndex, int width, int height);

private:
    void run();
    void signalHandler(int signal);
};
