#!/bin/bash

# Allow Docker to access X server
xhost +local:docker


# Detect system resources
TOTAL_MEM=$(free -g | awk '/^Mem:/{print $2}')
TOTAL_CPU=$(nproc)

# Allocate most resources for Gazebo (leave some for system)
MEM_LIMIT=$((TOTAL_MEM - 2))
CPU_LIMIT=$((TOTAL_CPU - 2))

# Minimum allocations
[ $MEM_LIMIT -lt 4 ] && MEM_LIMIT=4
[ $CPU_LIMIT -lt 2 ] && CPU_LIMIT=2

echo "======================================"
echo "System Resources:"
echo "  Total RAM: ${TOTAL_MEM}GB"
echo "  Total CPUs: ${TOTAL_CPU}"
echo "Allocating to Docker:"
echo "  RAM: ${MEM_LIMIT}GB"
echo "  CPUs: ${CPU_LIMIT}"
echo "======================================"

# Build docker command with high resource allocation
DOCKER_CMD="docker run -it --rm \
    --name bfmc \
    --privileged \
    --network host \
    \
    --memory=\"${MEM_LIMIT}g\" \
    --memory-swap=\"${MEM_LIMIT}g\" \
    --oom-kill-disable=false \
    --shm-size=\"8g\" \
    --cpus=\"${CPU_LIMIT}\" \
    --cpu-shares=1024 \
    --ulimit nofile=1024:524288 \
    \
    -e DISPLAY=\$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v ~/bfmc_simulator:/root/Documents \
    \
    -e LIBGL_ALWAYS_SOFTWARE=0 \
    -e QT_X11_NO_MITSHM=1"

# Detect and add GPU support
if command -v nvidia-smi &> /dev/null && nvidia-smi &> /dev/null; then
    echo "NVIDIA GPU detected"
    if docker run --rm --gpus all ubuntu:20.04 nvidia-smi &> /dev/null 2>&1; then
        echo "GPU acceleration enabled"
        DOCKER_CMD="$DOCKER_CMD --gpus all \
            --runtime=nvidia \
            -e NVIDIA_VISIBLE_DEVICES=all \
            -e NVIDIA_DRIVER_CAPABILITIES=all"
    else
        echo "NVIDIA GPU found but nvidia-container-toolkit not installed"
        echo "Install: sudo apt install nvidia-container-toolkit"
        echo "Then restart docker: sudo systemctl restart docker"
    fi
elif [ -d "/dev/dri" ]; then
    echo "Integrated graphics detected - enabling hardware acceleration"
    DOCKER_CMD="$DOCKER_CMD \
        --device=/dev/dri:/dev/dri \
        --group-add video \
        --group-add render"
else
    echo "No GPU detected - using software rendering"
fi

# Get the first IP address from hostname -I
HOST_IP=$(hostname -I | awk '{print $1}')

# Add ROS network configuration
DOCKER_CMD="$DOCKER_CMD \
    -e ROS_MASTER_URI=http://${HOST_IP}:11311 \
    -e ROS_IP=${HOST_IP} \
    bfmc_sim"

echo "======================================"
echo "ROS Network Configuration:"
echo "  ROS_MASTER_URI: http://${HOST_IP}:11311"
echo "  ROS_IP: ${HOST_IP}"
echo "======================================"
echo "Starting container..."
echo "======================================"

eval $DOCKER_CMD
