#!/bin/bash
# Script for use with xrdp remote desktop session
# ./run_bfmc_remote.sh              # Normal mode with GUI
# ./run_bfmc_remote.sh --headless   # No GUI (headless)
# ./run_bfmc_remote.sh --no-gpu     # Force software rendering

# Parse arguments
HEADLESS=false
USE_GPU=true
while [[ $# -gt 0 ]]; do
    case $1 in
        --headless)
            HEADLESS=true
            shift
            ;;
        --no-gpu)
            USE_GPU=false
            shift
            ;;
        *)
            echo "Unknown option: $1"
            echo "Usage: $0 [--headless] [--no-gpu]"
            exit 1
            ;;
    esac
done

# Check if DISPLAY is set (should be automatic in xrdp)
if [ "$HEADLESS" = "false" ]; then
    if [ -z "$DISPLAY" ]; then
        echo "======================================"
        echo "ERROR: DISPLAY not set!"
        echo "Are you logged in via xrdp?"
        echo "======================================"
        exit 1
    fi
    
    # Allow Docker to access X server
    xhost +local:docker 2>/dev/null
    DISPLAY_VAR="${DISPLAY}"
    
    echo "======================================"
    echo "Display Configuration:"
    echo "  Display: $DISPLAY_VAR"
    echo "  User: $USER"
    echo "======================================"
else
    DISPLAY_VAR=""
    echo "======================================"
    echo "Running in HEADLESS mode"
    echo "======================================"
fi

# Detect system resources
TOTAL_MEM=$(free -g | awk '/^Mem:/{print $2}')
TOTAL_CPU=$(nproc)

# Allocate 75% of resources
MEM_LIMIT=$((TOTAL_MEM * 3 / 4))
CPU_LIMIT=$((TOTAL_CPU * 3 / 4))

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

# Build docker command
DOCKER_CMD="docker run -it --rm \
    --name bfmc_rmt \
    --privileged \
    --network host \
    \
    --memory=\"${MEM_LIMIT}g\" \
    --memory-swap=\"$((MEM_LIMIT + 2))g\" \
    --shm-size=\"8g\" \
    --cpus=\"${CPU_LIMIT}\" \
    --cpu-shares=1024 \
    --ulimit nofile=1024:524288 \
    \
    -e DISPLAY=$DISPLAY_VAR \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v ~/bfmc_simulator:/root/Documents \
    \
    -e QT_X11_NO_MITSHM=1 \
    -e HEADLESS=$HEADLESS"

# Detect and add GPU support
if [ "$USE_GPU" = "true" ]; then
    if command -v nvidia-smi &> /dev/null && nvidia-smi &> /dev/null; then
        echo "NVIDIA GPU detected"
        if docker run --rm --gpus all ubuntu:20.04 nvidia-smi &> /dev/null 2>&1; then
            echo "GPU acceleration enabled"
            DOCKER_CMD="$DOCKER_CMD --gpus all \
                --runtime=nvidia \
                -e NVIDIA_VISIBLE_DEVICES=all \
                -e NVIDIA_DRIVER_CAPABILITIES=all \
                -e LIBGL_ALWAYS_SOFTWARE=0 \
                -e __GLX_VENDOR_LIBRARY_NAME=nvidia"
        else
            echo "NVIDIA GPU found but nvidia-container-toolkit not installed"
            echo "Install: sudo apt install nvidia-container-toolkit"
            echo "Then restart docker: sudo systemctl restart docker"
            echo "Falling back to software rendering"
            DOCKER_CMD="$DOCKER_CMD -e LIBGL_ALWAYS_SOFTWARE=1"
        fi
    elif [ -d "/dev/dri" ]; then
        echo "Integrated graphics detected - enabling hardware acceleration"
        DOCKER_CMD="$DOCKER_CMD \
            --device=/dev/dri:/dev/dri \
            --group-add video \
            --group-add render \
            -e LIBGL_ALWAYS_SOFTWARE=0"
    else
        echo "No GPU detected - using software rendering"
        DOCKER_CMD="$DOCKER_CMD -e LIBGL_ALWAYS_SOFTWARE=1"
    fi
else
    echo "GPU disabled by --no-gpu flag - using software rendering"
    DOCKER_CMD="$DOCKER_CMD -e LIBGL_ALWAYS_SOFTWARE=1"
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
