# Use the official ARM64 Debian base image
FROM --platform=linux/arm64 debian:bookworm AS build

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8

# Install required system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    ffmpeg \
    libevent-dev \
    libopencv-dev \
    libyaml-dev \
    cmake \
    gcc \
    g++ \
    curl \
    wget \
    sudo \
    openssh-server \
    build-essential \
    ca-certificates \
    meson \
    ninja-build \
    python3-yaml \
    python3-ply \
    python3-jinja2 \
    libgnutls28-dev \
    && rm -rf /var/lib/apt/lists/*

# Clone and build libcamera
RUN git clone https://git.libcamera.org/libcamera/libcamera.git \
    && cd libcamera \
    && meson setup build \
    && ninja -C build install \
    && ldconfig \
    && cd ..

# Modify the default SSH configuration
RUN ( \
    echo 'LogLevel DEBUG2'; \
    echo 'PermitRootLogin yes'; \
    echo 'PasswordAuthentication yes'; \
    echo 'Subsystem sftp /usr/lib/openssh/sftp-server'; \
  ) > /etc/ssh/sshd_config_test_clion \
  && mkdir /run/sshd

RUN sed -i 's/PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config

# Generate SSH host keys (if not already present)
RUN ssh-keygen -A

# Create a non-root user (for later file ownership or SSH login)
RUN useradd -m user && echo "user:password" | chpasswd && usermod -s /bin/bash user
RUN usermod -aG sudo root && usermod -aG sudo user
RUN chown -R user:user /home/user

EXPOSE 22

ENTRYPOINT service ssh start && bash

WORKDIR /home/user

# Clone the pi_stream project repository and fix ownership
RUN git clone --branch dev https://github.com/WIT-IEEE-MATE-ROV/pi_stream.git \
    && chown -R user:user /home/user/pi_stream
