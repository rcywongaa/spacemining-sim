FROM osrf/space-ros:latest
# ARG DEBIAN_FRONTEND=noninteractive # not visible in sudo commands

# Install dependencies

# Make sure the latest versions of packages are installed
# Using Docker BuildKit cache mounts for /var/cache/apt and /var/lib/apt ensures that
# the cache won't make it into the built image but will be maintained between steps.
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  sudo apt update && sudo DEBIAN_FRONTEND=noninteractive apt install -y \
    curl \
    git \
    libclang-dev \
    tmux \
    python3-pip \
    python3-pytest \
    g++ pkg-config libx11-dev libasound2-dev libudev-dev libxkbcommon-x11-0 mesa-vulkan-drivers \
    libx11-dev libxcursor-dev libxcb1-dev libxi-dev libxkbcommon-dev libxkbcommon-x11-dev \
    nvidia-driver-550 \
    vulkan-tools \
    && sudo rm -rf /var/lib/apt/lists/*

# Install Rust
USER ${USERNAME}
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
ENV PATH=${HOME_DIR}/.cargo/bin:$PATH
RUN cargo install cargo-ament-build

# RUN pip install --break-system-packages --upgrade pytest

# Install the colcon-cargo and colcon-ros-cargo plugins
RUN pip install git+https://github.com/colcon/colcon-cargo.git git+https://github.com/colcon/colcon-ros-cargo.git

RUN mkdir -p ${HOME_DIR}/workspace && echo "Did you forget to mount the repository into the Docker container?" > ${HOME_DIR}/workspace/HELLO.txt
WORKDIR ${HOME_DIR}/workspace
