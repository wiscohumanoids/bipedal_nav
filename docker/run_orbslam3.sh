#!/bin/bash
# Run the ORB-SLAM3 container, connected to the MuJoCo simulation via ROS2 topics.
#
# Prerequisites:
#   1. MuJoCo simulation must be running (either natively or in the humanoid_ws container)
#   2. The orbslam3_ros2 image must be built: docker build -f docker/Dockerfile.orbslam3 -t orbslam3_ros2:latest .
#
# Usage:
#   ./docker/run_orbslam3.sh                              # interactive shell
#   ./docker/run_orbslam3.sh ros2 launch orb_slam3_ros2_wrapper g1_rgbd.launch.py  # launch directly

set -e

IMAGE_NAME="orbslam3_ros2"
TAG="latest"
CONTAINER_NAME="orbslam3_dev"

# Allow X11 connections from Docker (for Pangolin viewer if enabled)
xhost +local:docker 2>/dev/null || true

ENV_SETUP="source /opt/ros/humble/setup.bash && source /root/colcon_ws/install/setup.bash && export LD_LIBRARY_PATH=/home/orb/ORB_SLAM3/lib:/home/orb/ORB_SLAM3/Thirdparty/DBoW2/lib:/home/orb/ORB_SLAM3/Thirdparty/g2o/lib:\${LD_LIBRARY_PATH}"

# Check if the container is already running
if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    if [ $# -eq 0 ]; then
        docker exec -it "${CONTAINER_NAME}" bash
    else
        docker exec -it "${CONTAINER_NAME}" bash -c "${ENV_SETUP} && $*"
    fi
else
    if [ $# -eq 0 ]; then
        # Interactive shell — .bashrc handles sourcing
        docker run --rm -it \
            --name "${CONTAINER_NAME}" \
            --network ros2_net \
            --privileged \
            -e DISPLAY="${DISPLAY:-:0}" \
            -e QT_X11_NO_MITSHM=1 \
            -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
            "${IMAGE_NAME}:${TAG}"
    else
        # Command mode — source workspace before running
        docker run --rm -it \
            --name "${CONTAINER_NAME}" \
            --network ros2_net \
            --privileged \
            -e DISPLAY="${DISPLAY:-:0}" \
            -e QT_X11_NO_MITSHM=1 \
            -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
            "${IMAGE_NAME}:${TAG}" \
            bash -c "${ENV_SETUP} && $*"
    fi
fi
