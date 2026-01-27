FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-catkin-tools \
    git \
    python3-pip \
    vim \
    wget \
    mesa-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    net-tools \
    iputils-ping \
    iproute2 \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN pip3 install numpy opencv-python matplotlib scipy pynput

# Create workspace
RUN mkdir -p /root/Documents
WORKDIR /root/Documents

# Setup ROS environment
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
RUN echo "source /root/Documents/Simulator/devel/setup.bash 2>/dev/null || true" >> /root/.bashrc

# Set Gazebo model path to include custom models
ENV GAZEBO_MODEL_PATH=/root/Documents/Simulator/src/models_pkg:/usr/share/gazebo-11/models
RUN echo "export GAZEBO_MODEL_PATH=/root/Documents/Simulator/src/models_pkg:/usr/share/gazebo-11/models" >> /root/.bashrc

# Gazebo performance and stability settings
ENV GAZEBO_MASTER_URI=http://localhost:11345

# OGRE rendering optimizations for stability
ENV OGRE_RTShader_Write=0
ENV OGRE_SKIP_XML_VALIDATION=1

# Gazebo resource and performance limits
ENV GAZEBO_MODEL_DATABASE_URI=""
ENV GAZEBO_RESOURCE_PATH=/root/Documents/Simulator/src/models_pkg
ENV GAZEBO_PLUGIN_PATH=/root/Documents/Simulator/devel/lib

# Rendering and GUI stability
ENV GAZEBO_GUI_INI_FILE=/root/.gazebo/gui.ini
ENV LIBGL_ALWAYS_INDIRECT=0

# Create .gazebo directory and gui.ini file to prevent warnings
RUN mkdir -p /root/.gazebo && \
    touch /root/.gazebo/gui.ini

# Set XDG_RUNTIME_DIR to prevent Qt warnings
ENV XDG_RUNTIME_DIR=/tmp/runtime-root
RUN mkdir -p /tmp/runtime-root && chmod 700 /tmp/runtime-root

# Add to bashrc
RUN echo "export GAZEBO_MASTER_URI=http://localhost:11345" >> /root/.bashrc
RUN echo "export OGRE_RTShader_Write=0" >> /root/.bashrc
RUN echo "export OGRE_SKIP_XML_VALIDATION=1" >> /root/.bashrc
RUN echo "export GAZEBO_MODEL_DATABASE_URI=\"\"" >> /root/.bashrc
RUN echo "export GAZEBO_RESOURCE_PATH=/root/Documents/Simulator/src/models_pkg" >> /root/.bashrc
RUN echo "export GAZEBO_PLUGIN_PATH=/root/Documents/Simulator/devel/lib" >> /root/.bashrc
RUN echo "export LIBGL_ALWAYS_INDIRECT=0" >> /root/.bashrc
RUN echo "export LIBGL_ALWAYS_INDIRECT=0" >> /root/.bashrc

# ROS network configuration (will be overridden by run_bfmc.sh)
# Set default localhost values, actual IP will be set at runtime
RUN echo '# ROS network configuration' >> /root/.bashrc
RUN echo 'if [ -z "$ROS_IP" ]; then' >> /root/.bashrc
RUN echo '  export ROS_IP=$(hostname -I | awk "{print \$1}")' >> /root/.bashrc
RUN echo '  export ROS_MASTER_URI=http://${ROS_IP}:11311' >> /root/.bashrc
RUN echo 'fi' >> /root/.bashrc

# Copy and setup startup script
COPY startup.sh /root/startup.sh
RUN chmod +x /root/startup.sh

# Run startup script which launches simulator in background and opens interactive shell
CMD ["/root/startup.sh"]
