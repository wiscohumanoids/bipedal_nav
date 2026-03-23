#!/bin/bash
# Run the humanoid_ws Docker container with display forwarding
#
# Usage:
#   ./docker/run.sh                  # interactive shell (or attach if already running)
#   ./docker/run.sh ros2 launch ...  # run a specific command
#
# If a container is already running, this script will exec into it
# instead of trying to create a second one.

set -e

IMAGE_NAME="humanoid_ws"
TAG="latest"
CONTAINER_NAME="humanoid_ws_dev"

# Allow X11 connections from Docker (needed for MuJoCo GUI)
xhost +local:docker 2>/dev/null || true

# Environment setup that sources the ROS2 workspace
ENV_SETUP="source /opt/ros/humble/setup.bash && source /home/ros2_ws/install/setup.bash && export LD_LIBRARY_PATH=/opt/mujoco/mujoco-3.2.7/lib:\${LD_LIBRARY_PATH}"

# Check if the container is already running
if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    if [ $# -eq 0 ]; then
        # Attach an interactive shell to the running container
        docker exec -it "${CONTAINER_NAME}" bash
    else
        # Run command inside the existing container
        docker exec -it "${CONTAINER_NAME}" bash -c "${ENV_SETUP} && $*"
    fi
else
    if [ $# -eq 0 ]; then
        # Interactive shell — .bashrc handles sourcing
        docker run --rm -it \
            --name "${CONTAINER_NAME}" \
            --network host \
            --privileged \
            -e DISPLAY="${DISPLAY:-:0}" \
            -e QT_X11_NO_MITSHM=1 \
            -e WAYLAND_DISPLAY="${WAYLAND_DISPLAY}" \
            -e XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR}" \
            -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
            -v "${XDG_RUNTIME_DIR:-/tmp}":"${XDG_RUNTIME_DIR:-/tmp}" \
            "${IMAGE_NAME}:${TAG}"
    else
        # Command mode — source workspace before running
        docker run --rm -it \
            --name "${CONTAINER_NAME}" \
            --network host \
            --privileged \
            -e DISPLAY="${DISPLAY:-:0}" \
            -e QT_X11_NO_MITSHM=1 \
            -e WAYLAND_DISPLAY="${WAYLAND_DISPLAY}" \
            -e XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR}" \
            -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
            -v "${XDG_RUNTIME_DIR:-/tmp}":"${XDG_RUNTIME_DIR:-/tmp}" \
            "${IMAGE_NAME}:${TAG}" \
            bash -c "${ENV_SETUP} && $*"
    fi
fi
