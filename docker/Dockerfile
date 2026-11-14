# =============================================================================
# visual_haptics_vla: ROS 2 Jazzy + Gazebo Harmonic + PyTorch (GTX 1650, 4 GB)
# Build:  docker compose build      Run:  docker compose up -d && docker compose exec sim bash
# =============================================================================
FROM osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1

# --- 1. SYSTEM + ROS 2 PACKAGES (unchanged: stays cached) --------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-ros-gz \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-gripper-controllers \
    ros-jazzy-cv-bridge \
    ros-jazzy-message-filters \
    ros-jazzy-dynamixel-sdk \
    mesa-utils \
    python3-venv \
    python3-colcon-common-extensions \
    git \
    build-essential \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# --- 2. PYTHON VENV (unchanged: stays cached) --------------------------------
RUN python3 -m venv --system-site-packages /opt/venv
ENV PATH="/opt/venv/bin:$PATH"

# --- 3. ML STACK, PINNED ------------------------------------------------------
# Pinned for reproducibility. cu121 tops out at torch 2.5.1, which is what you
# already downloaded, so the pip cache mount makes this layer fast.
# Verify first inside the old container: pip show torch torchvision
RUN --mount=type=cache,target=/root/.cache/pip \
    pip install --upgrade pip && \
    pip install torch==2.5.1 torchvision==0.20.1 \
        --index-url https://download.pytorch.org/whl/cu121 && \
    pip install onnx onnxruntime-gpu sentence-transformers "numpy<2"

# --- 4. ROBOTIS OPENMANIPULATOR-X UNDERLAY (built as root, read-only for user)
# After the first good build, replace each branch with the commit hash from
# `git -C /opt/robotis_ws/src/<repo> rev-parse HEAD` so rebuilds are identical.
#ARG OM_REF=jazzy
#ARG DXL_HW_REF=jazzy
#ARG RB_IF_REF=main
# Pinned to the commits of the first working build (12 Nov 2026):
# open_manipulator @ jazzy, dynamixel_hardware_interface @ jazzy, robotis_interfaces @ main
ARG OM_REF=17efe42bd1df178fa44d49c43250fc4304343dbb
ARG DXL_HW_REF=561376033c018b4248917cb65aaacf511193980f
ARG RB_IF_REF=c79afb6aa8d0900401ec38ecdb60b1a104929997


WORKDIR /opt/robotis_ws
RUN mkdir -p src && cd src && \
    git clone https://github.com/ROBOTIS-GIT/open_manipulator.git && \
    git -C open_manipulator checkout ${OM_REF} && \
    git clone https://github.com/ROBOTIS-GIT/dynamixel_hardware_interface.git && \
    git -C dynamixel_hardware_interface checkout ${DXL_HW_REF} && \
    git clone https://github.com/ROBOTIS-GIT/robotis_interfaces.git && \
    git -C robotis_interfaces checkout ${RB_IF_REF}

RUN . /opt/ros/jazzy/setup.sh && \
    apt-get update && \
    # Jazzy index only (much smaller), with retries: raw.githubusercontent.com
    # can time out (build failure 14 Nov).
    for i in 1 2 3; do rosdep update --rosdistro jazzy && break || sleep 15; done && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    rm -rf /var/lib/apt/lists/*

# --- 5. NON-ROOT USER (UID/GID 1000 = your WSL user) -------------------------
# Only /workspace is chowned. Chowning /opt/venv (~5 GB) copies every file
# into a new layer and doubles the image size.
# Need a new pip package later? Use: sudo /opt/venv/bin/pip install <pkg>
ARG USERNAME=pkm
ARG USER_UID=1000
ARG USER_GID=1000

RUN userdel -r ubuntu 2>/dev/null || true && \
    groupadd --gid ${USER_GID} ${USERNAME} && \
    useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ${USERNAME} && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} && \
    chmod 0440 /etc/sudoers.d/${USERNAME} && \
    mkdir -p /workspace/ros2_ws/src /workspace/ros2_ws/build \
             /workspace/ros2_ws/install /workspace/ros2_ws/log /workspace/data && \
    chown -R ${USER_UID}:${USER_GID} /workspace

# --- 6. ENVIRONMENT SOURCING --------------------------------------------------
# Entrypoint covers non-interactive commands (docker compose exec sim <cmd>);
# .bashrc covers interactive shells and also picks up your own overlay.
RUN printf '%s\n' \
    '#!/bin/bash' \
    'set -e' \
    'source /opt/ros/jazzy/setup.bash' \
    'source /opt/robotis_ws/install/setup.bash' \
    '[ -f /workspace/ros2_ws/install/setup.bash ] && source /workspace/ros2_ws/install/setup.bash' \
    'exec "$@"' > /entrypoint.sh && chmod +x /entrypoint.sh

USER ${USERNAME}

RUN printf '%s\n' \
    'source /opt/ros/jazzy/setup.bash' \
    'source /opt/robotis_ws/install/setup.bash' \
    '[ -f /workspace/ros2_ws/install/setup.bash ] && source /workspace/ros2_ws/install/setup.bash' \
    "alias cb='cd /workspace/ros2_ws && colcon build --symlink-install && source install/setup.bash'" \
    '[ -f /workspace/ros2_ws/src/rt1_sim_bringup/scripts/sim_helpers.sh ] && source /workspace/ros2_ws/src/rt1_sim_bringup/scripts/sim_helpers.sh' \
    >> ~/.bashrc

WORKDIR /workspace
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]