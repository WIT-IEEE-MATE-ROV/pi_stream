cmake_minimum_required(VERSION 3.2)
project(pi_stream)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS} include)
add_executable(Stream src/main.cpp src/client.cpp)
target_link_libraries(Stream ${OpenCV_LIBS})
