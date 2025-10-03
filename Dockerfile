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

# Install eCAL using Vilota software installer
COPY src/VilotaSoftwareInstaller_v2.0-bba1a93f.py /tmp/installer.py
# Ensure helper tools needed by the Vilota installer are available during build
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        python3-requests \
        software-properties-common \
        sudo \
        iputils-ping \
        apt-transport-https \
        ca-certificates \
        gnupg && \
    rm -rf /var/lib/apt/lists/*

# Install eCAL, capnp and dependencieswith Vilota installer (requires credentials to be passed as build args)
RUN apt-get update && \
    python3 /tmp/installer.py -s -u deployment -p vilota-dep --no-update-script && \
    rm -rf /var/lib/apt/lists/*

# Install eCAL Python bindings explicitly
RUN apt-get update && \
    apt-get install -y --no-install-recommends python3-ecal5 && \
    rm -rf /var/lib/apt/lists/* || \
    echo "Warning: python3-ecal5 package not available"

# Verify eCAL and pycapnp installation
RUN python3 -c "import ecal; print('eCAL imported successfully')" || echo "Warning: eCAL import failed"
RUN python3 -c "import capnp; print(f'pycapnp {capnp.__version__} imported successfully')" || echo "Warning: pycapnp import failed"

# Ensure udev rules directory exists and add Movidius udev rule so installer
# doesn't fail if /etc/udev is missing inside minimal images. We don't run
# `udevadm` here because udev may not be active during image build; the
# rule will be present at runtime and can be reloaded by the host/container
# init if needed.
RUN mkdir -p /etc/udev/rules.d && \
    echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' > /etc/udev/rules.d/80-movidius.rules || true

# Install eCAL and dependencies for odom.py
# RUN apt-get update && apt-get install -y --no-install-recommends \
#     capnproto \
#     libcapnp-dev \
#     python3-dev \
#     && rm -rf /var/lib/apt/lists/*

# Install other Python dependencies
# RUN pip3 install --no-cache-dir \
#     numpy \
#     scipy \
#     matplotlib \
#     argparse

# Install Python dependencies
# RUN pip3 install --no-cache-dir --upgrade pip setuptools wheel

# Try to install pycapnp 2.x first (has pre-built wheels)
# If it fails due to Cython conflicts, fall back to older version
# RUN pip3 install --no-cache-dir "pycapnp>=2.0.0" || \
#     (echo "pycapnp 2.x failed, trying with Cython upgrade..." && \
#      pip3 install --no-cache-dir --force-reinstall "Cython>=3.0" && \
#      pip3 install --no-cache-dir "pycapnp>=2.0.0") || \
#     (echo "Falling back to pycapnp 1.x..." && \
#      pip3 install --no-cache-dir "Cython<3" && \
#      pip3 install --no-cache-dir "pycapnp==1.3.0") || \
#     echo "Warning: pycapnp installation failed, odom.py may not work"

# Install MAVROS geographic datasets
RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh \
    && bash ./install_geographiclib_datasets.sh \
    && rm install_geographiclib_datasets.sh

# Enable the use of `source` keyword
SHELL ["/bin/bash", "-c"]

# Create workspace and copy source code
WORKDIR /catkin_ws/src/
COPY . /catkin_ws/src/shiviz_px4_nav/

# Copy eCAL configuration
COPY src/ecal.ini /etc/ecal/ecal.ini

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