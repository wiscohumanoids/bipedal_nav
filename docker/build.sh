#!/bin/bash
# Build the humanoid_ws Docker image
#
# Usage:
#   ./docker/build.sh              # default (x86_64, MuJoCo 3.2.7)
#   ./docker/build.sh --no-cache   # rebuild from scratch

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

IMAGE_NAME="humanoid_ws"
TAG="latest"
ROS_DISTRO="humble"
MUJOCO_VERSION="3.2.7"

# Detect CPU architecture
ARCH="$(uname -m)"
CPU_ARCH="x86_64"
if [[ "$ARCH" == "aarch64" || "$ARCH" == "arm64" ]]; then
    CPU_ARCH="aarch64"
fi

echo "Building ${IMAGE_NAME}:${TAG}"
echo "  ROS: ${ROS_DISTRO} | MuJoCo: ${MUJOCO_VERSION} | Arch: ${CPU_ARCH}"

docker build \
    --build-arg ROS_DISTRO="${ROS_DISTRO}" \
    --build-arg MUJOCO_VERSION="${MUJOCO_VERSION}" \
    --build-arg CPU_ARCH="${CPU_ARCH}" \
    -f "${REPO_ROOT}/Dockerfile" \
    -t "${IMAGE_NAME}:${TAG}" \
    "$@" \
    "${REPO_ROOT}"
