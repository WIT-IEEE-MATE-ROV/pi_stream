#include <iostream>
#include <print>
#include <string>

#include <chrono>
#include <thread>
// #include <opencv2/opencv.hpp>

#include "udpclient.h"


using namespace std;

int main()
{
    const char *msg = "Test\0";
  while (true)
  {
  
    bool ret = udpSend(msg);
    if (!ret)
      std::printf("Failed to send message");

    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::printf("Message %s sent\n", msg);
  }
  return 0;
}