ARG ROS_DISTRO=humble
FROM ros:$ROS_DISTRO as base
ARG DEBIAN_FRONTEND=noninteractive

# Install dependencies
RUN apt-get update && apt-get install -y \
    curl \
    git \
    libclang-dev \
    tmux \
    python3-pip \
    python3-pytest \
    g++ pkg-config libx11-dev libasound2-dev libudev-dev libxkbcommon-x11-0 mesa-vulkan-drivers\
    libx11-dev libxcursor-dev libxcb1-dev libxi-dev libxkbcommon-dev libxkbcommon-x11-dev \
    nvidia-driver-550 \
    vulkan-tools \
    && rm -rf /var/lib/apt/lists/*

# Install Rust
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
ENV PATH=/root/.cargo/bin:$PATH

# RUN pip install --break-system-packages --upgrade pytest

# Install the colcon-cargo and colcon-ros-cargo plugins
RUN pip install --break-system-packages git+https://github.com/colcon/colcon-cargo.git git+https://github.com/colcon/colcon-ros-cargo.git

RUN mkdir -p /workspace && echo "Did you forget to mount the repository into the Docker container?" > /workspace/HELLO.txt
WORKDIR /workspace
