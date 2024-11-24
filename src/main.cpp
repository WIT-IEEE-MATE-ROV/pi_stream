#include <iostream>
#include <string>
#include <ctime>
#include <chrono>
#include <thread>
#include <opencv2/opencv.hpp>
#include "udpclient.h"

using namespace std;

int main()
{

  std::string ipAddress = "127.0.0.1"; // Replace with the receiver's IP address
  int port = 8080;                     // Replace with the desired UDP port
  int cameraIndex = 0;                 // Default camera index


  // I have to set resloution to 320x180 rn so frame can fit in UDP packet 
  streamVideoOverUDP(ipAddress, port, cameraIndex, 320, 180);

  return 0;
}