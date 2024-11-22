#include <stdio.h>
#include <iostream>
#include <chrono>
#include <ctime>
#include <sstream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <stdlib.h>

#include "udpclient.h"

// #include <opencv.hpp>

std::string getCurrentTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);

    // Format the time
    std::ostringstream ss;
    ss << std::time_put(std::localtime(&in_time_t), "%Y-%m-%d %H:%M:%S");
    return ss.str();
}


bool udpSend(const char *msg){
    sockaddr_in servaddr;
    int fd = socket(AF_INET,SOCK_DGRAM,0);
    if(fd<0){
        perror("cannot open socket");
        return false;
    }
    
    bzero(&servaddr,sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = inet_addr(HOSTNAME);
    servaddr.sin_port = htons(PORT);
    // replace with opencv mat (frame)
    if (sendto(fd, msg, strlen(msg)+1, 0, 
               (sockaddr*)&servaddr, sizeof(servaddr)) < 0){
        perror("cannot send message");
        close(fd);
        return false;
    }
    close(fd);
    //std::printf("\tSomething happeend in client.cpp\n");
    return true;
 }
