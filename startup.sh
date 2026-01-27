#!/bin/bash

# Source ROS environment
source /opt/ros/noetic/setup.bash

# Source workspace setup if it exists
if [ -f /root/Documents/Simulator/devel/setup.bash ]; then
    source /root/Documents/Simulator/devel/setup.bash
fi

# Set VirtualGL display
export VGL_DISPLAY=:0

# Launch the simulator in background WITH VirtualGL
echo "======================================"
echo "Starting BFMC Simulator with VirtualGL..."
echo "======================================"

# Check if VirtualGL is available
if command -v vglrun &>/dev/null; then
    echo "Using VirtualGL for GPU acceleration"
    vglrun roslaunch sim_pkg map_with_all_objects_REC.launch &
else
    echo "VirtualGL not found, launching without GPU acceleration"
    roslaunch sim_pkg map_with_all_objects_REC.launch &
fi

# Wait a moment for roslaunch to initialize
sleep 3

echo "======================================"
echo "Simulator launched in background"
echo "ROS Master: $ROS_MASTER_URI"
echo "Display: $DISPLAY"
echo "VGL Display: $VGL_DISPLAY"
echo "======================================"
echo ""
echo "Useful commands:"
echo "  rostopic list          - List all topics"
echo "  rosnode list           - List all nodes"
echo "  rqt_graph              - Visualize node graph"
echo "  vglrun gzclient        - Restart Gazebo GUI with GPU"
echo "  pkill -f roslaunch     - Stop the simulator"
echo "======================================"

# Start interactive bash shell
exec /bin/bash
