#pragma once
#ifndef CLIENT_H
#define CLIENT_H

#define HOSTNAME "127.0.0.1"
#define PORT 8080

std::string getCurrentTimestamp();
bool udpSend(const char *msg);

#endif