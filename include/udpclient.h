#pragma once
#ifndef CLIENT_H
#define CLIENT_H

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

std::string getCurrentTimestamp();
void streamVideoOverUDP(const std::string &ipAddress, int port, int cameraIndex, int width, int height);

#endif