# Use the NVIDIA Isaac Sim image as the base
FROM nvcr.io/nvidia/isaac-sim:4.2.0

# Set environment variables for non-interactive installation
ENV DEBIAN_FRONTEND=noninteractive

# Update and install prerequisites for ROS 2 Humble
RUN apt-get update && apt-get install -y \
    software-properties-common \
    curl \
    gnupg2 \
    lsb-release && \
    rm -rf /var/lib/apt/lists/*

# Add ROS 2 repository and keys
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    add-apt-repository "deb [arch=amd64] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main"

# Install ROS 2 Humble
RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    python3-argcomplete && \
    rm -rf /var/lib/apt/lists/*

# Source ROS 2 setup script in the container
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Ensure NVIDIA container runtime is set up correctly
ENV NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=all

# add the installation of colcon and rosdep
RUN apt-get update
RUN apt install python3-colcon-common-extensions -y
RUN apt install python3-rosdep -y && \
    rosdep init && \
    rosdep update


# Override the entrypoint with a shell to prevent automatic script execution
ENTRYPOINT ["/bin/bash"]

# Set default command to bash
# CMD ["/bin/bash"]