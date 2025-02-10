cmake_minimum_required(VERSION 3.16)
project(pi_stream)

# Find OpenCV
find_package(OpenCV REQUIRED)

# Use pkg-config for dependencies
find_package(PkgConfig REQUIRED)
pkg_check_modules(AVCODEC REQUIRED libavcodec)
pkg_check_modules(AVFORMAT REQUIRED libavformat)
pkg_check_modules(AVUTIL REQUIRED libavutil)
pkg_check_modules(LIBCAMERA REQUIRED libcamera libcamera-base)  # Modified
pkg_check_modules(LIBEVENT REQUIRED libevent libevent_pthreads)  # Modified
pkg_check_modules(YAML REQUIRED yaml-0.1)
# Include directories
include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
    ${AVCODEC_INCLUDE_DIRS}
    ${AVFORMAT_INCLUDE_DIRS}
    ${AVUTIL_INCLUDE_DIRS}
    ${LIBCAMERA_INCLUDE_DIRS}
    ${LIBEVENT_INCLUDE_DIRS}
    ${YAML_INCLUDE_DIRS}
    src/apps/cam
)

# Add executables
add_executable(Stream src/main.cpp src/client.cpp)

add_executable(Example 
    src/apps/cam/main.cpp
    src/apps/common/options.cpp    
    src/apps/common/event_loop.cpp
    # Add these if they exist in your project:
    src/apps/cam/camera_session.cpp  # Add if you have this file
    src/apps/cam/file_sink.cpp
    src/apps/cam/frame_sink.cpp
    src/apps/cam/capture_script.cpp
    src/apps/common/ppm_writer.cpp
    src/apps/common/image.cpp
    src/apps/common/stream_options.cpp  # Add if you have this file
)

target_link_libraries(Stream PRIVATE
    ${OpenCV_LIBS}
    ${AVCODEC_LIBRARIES}
    ${AVFORMAT_LIBRARIES}
    ${AVUTIL_LIBRARIES}
    ${LIBCAMERA_LIBRARIES}
    ${LIBEVENT_LIBRARIES}
    ${LIBCAMERA_BASE_LIB}
    ${LIBCAMERA_LIB}
    ${LIBCAMERA_APP_LIB}
    ${YAML_LIBRARIES}
)

# Link libraries
target_link_libraries(Example PRIVATE
    ${LIBCAMERA_LIBRARIES}
    ${LIBEVENT_LIBRARIES}
    ${OpenCV_LIBS}
    ${AVCODEC_LIBRARIES}
    ${AVFORMAT_LIBRARIES}
    ${AVUTIL_LIBRARIES}
    ${YAML_LIBRARIES}
    pthread  # Explicitly link pthread
)

# Set RPATH if needed
target_link_options(Example PRIVATE "-Wl,-rpath=/usr/lib/aarch64-linux-gnu")

