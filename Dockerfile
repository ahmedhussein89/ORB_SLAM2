FROM ubuntu:18.04

# Prevent interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=UTC

# Set working directory
WORKDIR /workspace

# Install system dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    libopencv-dev \
    python3-dev \
    libeigen3-dev \
    libglew-dev \
    libboost-dev \
    && rm -rf /var/lib/apt/lists/*

# Install Pangolin
RUN cd /tmp && \
    git clone --depth 1 --branch v0.6 https://github.com/stevenlovegrove/Pangolin.git && \
    cd Pangolin && \
    mkdir build && \
    cd build && \
    cmake \
        -DCMAKE_BUILD_TYPE=Release \
         -DCPP11_NO_BOOST=1 \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        .. && \
    make -j$(nproc) && \
    make install && \
    ldconfig && \
    rm -rf /tmp/Pangolin

# Copy the ORB-SLAM2 source code
COPY . /workspace/ORB_SLAM2

# Set working directory to ORB-SLAM2
WORKDIR /workspace/ORB_SLAM2

# Build the project
RUN bash /workspace/ORB_SLAM2/build.sh

