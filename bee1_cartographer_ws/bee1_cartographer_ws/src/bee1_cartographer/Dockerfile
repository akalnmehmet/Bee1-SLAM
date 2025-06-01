# Beemobs Bee1 Cartographer System Docker Image
FROM ros:humble-desktop

# Set environment variables
ENV ROS_DISTRO=humble
ENV RMW_IMPLEMENTATION=rmw_cyclonedx_cpp
ENV ROS_DOMAIN_ID=42

# Install dependencies
RUN apt-get update && apt-get install -y \
    ros-humble-cartographer \
    ros-humble-cartographer-ros \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-robot-localization \
    ros-humble-velodyne \
    ros-humble-velodyne-driver \
    ros-humble-velodyne-pointcloud \
    ros-humble-pointcloud-to-laserscan \
    ros-humble-ublox \
    ros-humble-xacro \
    ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher \
    ros-humble-tf2-tools \
    ros-humble-tf2-geometry-msgs \
    python3-geojson \
    python3-pyproj \
    python3-colcon-common-extensions \
    python3-rosdep \
    git \
    wget \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
WORKDIR /workspace
COPY . /workspace/src/bee1_cartographer/

# Install rosdep dependencies
RUN rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

# Build workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

# Setup entrypoint
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Create bee1 user
RUN useradd -m -s /bin/bash bee1 && \
    chown -R bee1:bee1 /workspace

USER bee1

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]