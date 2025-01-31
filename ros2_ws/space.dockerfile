FROM osrf/space-ros:main
# ARG DEBIAN_FRONTEND=noninteractive # not visible in sudo commands

# Install Rust
USER ${USERNAME}
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
ENV PATH=${HOME_DIR}/.cargo/bin:$PATH

# Install ros2-rust dependencies
RUN cargo install cargo-ament-build
RUN pip install git+https://github.com/colcon/colcon-cargo.git git+https://github.com/colcon/colcon-ros-cargo.git

# Install dependencies
# Using Docker BuildKit cache mounts for /var/cache/apt and /var/lib/apt ensures that
# the cache won't make it into the built image but will be maintained between steps.
# RUN --mount=type=cache,target=/var/cache/apt,sharing=locked --mount=type=cache,target=/var/lib/apt,sharing=locked \
#   sudo apt update
# Do apt update and apt install together to prevent stale apt cache
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked --mount=type=cache,target=/var/lib/apt,sharing=locked \
  sudo apt update && sudo DEBIAN_FRONTEND=noninteractive apt install -y \
    curl \
    git \
    libclang-dev \
    tmux \
    python3-pip \
    python3-pytest \
    # For bevy
    g++ pkg-config libx11-dev libasound2-dev libudev-dev libxkbcommon-x11-0 mesa-vulkan-drivers \
    libx11-dev libxcursor-dev libxcb1-dev libxi-dev libxkbcommon-dev libxkbcommon-x11-dev \
    nvidia-driver-550 \
    vulkan-tools \
    # For BehaviorTree.CPP
    libczmq-dev \
    # For BehaviorTree.ROS2
    python3-jinja2 python3-typeguard \
    # Dependencies
    libgeographic-dev

# Bind mount ensures rosdep finds dependencies
# RUN --mount=type=cache,target=/var/cache/apt,sharing=locked --mount=type=cache,target=/var/lib/apt,sharing=locked \
#   --mount=type=bind,target=${HOME_DIR}/workspace \
#   rosdep update && rosdep install --from-paths ${HOME_DIR}/workspace/src --ignore-src -r -y

# Get rosinstall_generator
# RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
#   --mount=type=cache,target=/var/lib/apt,sharing=locked \
#   sudo apt-get update -y && sudo apt-get install -y python3-rosinstall-generator

# Generate repos file for dependencies, excluding packages from Space ROS core.
# COPY --chown=${USERNAME}:${USERNAME} extra-pkgs.txt /tmp/
# COPY --chown=${USERNAME}:${USERNAME} excluded-pkgs.txt /tmp/
# RUN rosinstall_generator \
#   --rosdistro ${ROS_DISTRO} \
#   --deps \
#   --exclude-path ${SPACEROS_DIR}/src \
#   --exclude $(cat /tmp/excluded-pkgs.txt) -- \
#   -- $(cat /tmp/moveit2-pkgs.txt) \
#   > /tmp/generated_pkgs.repos
# RUN vcs import src < /tmp/generated_pkgs.repos

# Reduce image size
# RUN sudo rm -rf /var/lib/apt/lists/*

# RUN pip install --break-system-packages --upgrade pytest

RUN mkdir -p ${HOME_DIR}/workspace && echo "Did you forget to mount the repository into the Docker container?" > ${HOME_DIR}/workspace/HELLO.txt
WORKDIR ${HOME_DIR}/workspace
