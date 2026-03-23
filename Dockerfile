ARG ROS_DISTRO=humble

FROM ros:${ROS_DISTRO}

ARG MUJOCO_VERSION=3.2.7
ARG CPU_ARCH=x86_64

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# ── System dependencies ─────────────────────────────────────────────────────
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y --no-install-recommends \
        python3-pip \
        python3-venv \
        wget \
        git \
        cmake \
        build-essential \
        libglfw3-dev \
        libeigen3-dev \
        libgl1-mesa-dev \
        libglu1-mesa-dev \
        libosmesa6-dev \
        mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# ── ROS2 packages ───────────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-ros2-control \
        ros-${ROS_DISTRO}-ros2-controllers \
        ros-${ROS_DISTRO}-joint-state-broadcaster \
        ros-${ROS_DISTRO}-joint-trajectory-controller \
        ros-${ROS_DISTRO}-effort-controllers \
        ros-${ROS_DISTRO}-robot-state-publisher \
        ros-${ROS_DISTRO}-xacro \
        ros-${ROS_DISTRO}-tf2-ros \
        ros-${ROS_DISTRO}-controller-manager \
        python3-colcon-common-extensions \
        python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# ── Install MuJoCo C library ────────────────────────────────────────────────
ENV MUJOCO_VERSION=${MUJOCO_VERSION}
ENV MUJOCO_DIR="/opt/mujoco/mujoco-${MUJOCO_VERSION}"

RUN mkdir -p /opt/mujoco && \
    wget -q "https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/mujoco-${MUJOCO_VERSION}-linux-${CPU_ARCH}.tar.gz" -O /tmp/mujoco.tar.gz && \
    tar -xzf /tmp/mujoco.tar.gz -C /opt/mujoco && \
    rm /tmp/mujoco.tar.gz

# ── Create Python virtual environment ───────────────────────────────────────
ENV VENV_PATH="/opt/venv"
RUN python3 -m venv --system-site-packages ${VENV_PATH}
ENV PATH="${VENV_PATH}/bin:${PATH}"

RUN pip install --no-cache-dir \
        xacro \
        numpy \
        scipy \
        opencv-python-headless

# ── Copy workspace source ──────────────────────────────────────────────────
ENV ROS_WS="/home/ros2_ws"
RUN mkdir -p ${ROS_WS}/src
WORKDIR ${ROS_WS}

COPY src/ src/

# ── Install any remaining ROS dependencies via rosdep ───────────────────────
RUN rosdep update && \
    rosdep install --from-paths src/ --ignore-src -y 2>/dev/null || true

# ── Build the workspace ────────────────────────────────────────────────────
RUN . /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DMUJOCO_HEADLESS_OSMESA=ON

# ── Shell setup ─────────────────────────────────────────────────────────────
RUN echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> /root/.bashrc && \
    echo 'source ${ROS_WS}/install/setup.bash' >> /root/.bashrc && \
    echo 'source ${VENV_PATH}/bin/activate' >> /root/.bashrc && \
    echo 'export LD_LIBRARY_PATH=${MUJOCO_DIR}/lib:${LD_LIBRARY_PATH}' >> /root/.bashrc

CMD ["/bin/bash"]
