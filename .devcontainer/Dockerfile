FROM ubuntu:18.04

# Configure apt to not install suggested or recommended packages
RUN echo 'APT::Install-Suggests "0";' >> /etc/apt/apt.conf.d/00-docker && \
    echo 'APT::Install-Recommends "0";' >> /etc/apt/apt.conf.d/00-docker

# Set the ARG for non-interactive apt
ARG DEBIAN_FRONTEND=noninteractive

# Install all dependencies in one step to reduce layers
RUN apt-get update &&  \
    apt-get install -y \
    build-essential    \
    cmake              \
    libboost-all-dev   \
    libeigen3-dev      \
    libglew-dev        \
    libopencv-dev      \
    locales            \
    ninja-build        \
    python3-venv       \
    wget

# Generate en_US.UTF-8 locale on the system
RUN locale-gen en_US.UTF-8

RUN wget -q https://github.com/stevenlovegrove/Pangolin/archive/refs/tags/v0.6.tar.gz -O /tmp/Pangolin.tar.gz && \
    mkdir -p /tmp/Pangolin/build && \
    tar -xzf /tmp/Pangolin.tar.gz -C /tmp/Pangolin --strip-components=1 && \
    cd /tmp/Pangolin/build && \
    cmake -G Ninja -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF -DBUILD_TOOLS=OFF -DBUILD_EXAMPLES=OFF .. && \
    ninja install && \
    rm -rf /tmp/Pangolin.tar.gz /tmp/Pangolin

# Add a non-root user in one step
RUN useradd -ms /bin/bash vscode \
    && cp -r /root/.bashrc /home/vscode/ \
    && cp -r /root/.profile /home/vscode/ \
    && chown vscode:vscode /home/vscode/.bashrc /home/vscode/.profile

USER vscode
