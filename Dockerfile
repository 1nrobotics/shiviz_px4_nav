FROM ros:noetic-ros-core-focal
LABEL maintainer="1nrobotics@github"
LABEL description="Docker image for shiviz_px4_nav - PX4 Offboard Navigation with Waypoint Following"

# Set environment variables
ENV ROS_DISTRO=noetic
ENV DEBIAN_FRONTEND=noninteractive

# Install bootstrap tools and essential packages
RUN apt-get update && apt-get install --no-install-recommends -y \
    vim \
    wget \
    curl \
    git \
    tmux \
    iproute2 \
    build-essential \
    python3-rosdep \
    python3-rosinstall \
    python3-vcstools \
    python3-catkin-tools \
    python3-pip \
    xmlstarlet \
    && rm -rf /var/lib/apt/lists/*

# Bootstrap rosdep
RUN rosdep init && \
    rosdep update --rosdistro $ROS_DISTRO

# Install ROS packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-ros-base=1.5.0-1* \
    && rm -rf /var/lib/apt/lists/*

# Install MAVROS and related dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-mavlink \
    ros-${ROS_DISTRO}-mavros \
    ros-${ROS_DISTRO}-mavros-msgs \
    ros-${ROS_DISTRO}-mavros-extras \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-tf2-geometry-msgs \
    && rm -rf /var/lib/apt/lists/*

# Install additional ROS packages for navigation and control
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-pcl-conversions \
    ros-${ROS_DISTRO}-eigen-conversions \
    ros-${ROS_DISTRO}-cmake-modules \
    && rm -rf /var/lib/apt/lists/*

# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    libeigen3-dev \
    libpcl-dev \
    libyaml-cpp-dev \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
# NOTE: pycapnp is commented out due to ARM64 compilation issues
# Uncomment and install separately if eCAL integration is needed
RUN pip3 install --no-cache-dir \
    numpy \
    scipy \
    matplotlib \
    argparse
    # pycapnp

# Install MAVROS geographic datasets
RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh \
    && bash ./install_geographiclib_datasets.sh \
    && rm install_geographiclib_datasets.sh

# Enable the use of `source` keyword
SHELL ["/bin/bash", "-c"]

# Create workspace and copy source code
WORKDIR /catkin_ws/src/
COPY . /catkin_ws/src/shiviz_px4_nav/

# Initialize and build the workspace
WORKDIR /catkin_ws/
RUN source /opt/ros/noetic/setup.bash \
    && catkin init \
    && catkin config --extend /opt/ros/noetic \
    && catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -r -y \
    && catkin build

# Set up environment in bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "source /catkin_ws/devel/setup.bash" >> /root/.bashrc && \
    echo "export ROS_MASTER_URI=http://localhost:11311" >> /root/.bashrc && \
    echo "export ROS_HOSTNAME=localhost" >> /root/.bashrc && \
    echo "export ROS_IP=localhost" >> /root/.bashrc

# Copy custom bashrc for additional configuration
COPY docker/.bashrc /root/.bashrc_custom
RUN cat /root/.bashrc_custom >> /root/.bashrc

# Set working directory
WORKDIR /catkin_ws

# Expose ROS and MAVROS ports
EXPOSE 11311 14540 14557

# Default environment variables (can be overridden at runtime)
ENV PX4_SIM_HOST=localhost
ENV PX4_SIM_MODEL=iris
ENV VEHICLE_ID=1

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]